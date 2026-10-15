#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <thread>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include <chrono>
#include <memory>
#include <string>
#include <exception>

using namespace std;
using GoalHandleMoveToWaypoint =
    rclcpp_action::ServerGoalHandle<museum_docent_robot::action::MoveToWaypoint>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Movement : public rclcpp::Node {
private:
    enum class MovePhase { Idle, Reserved, Active, Canceling };

    struct MoveContext {
        shared_ptr<GoalHandleMoveToWaypoint> move_goal;
        GoalHandleNav2::SharedPtr nav2_goal;
    };

    mutex goal_mutex_;
    MovePhase phase_ = MovePhase::Idle;
    MoveContext current_;
    rclcpp_action::Server<museum_docent_robot::action::MoveToWaypoint>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    string package_path = ament_index_cpp::get_package_share_directory("museum_docent_robot");

public:
    // 기존 MoveToWaypoint Action Server와 Nav2 Client 생성
    Movement() : rclcpp::Node("movement") {
        action_server_ = rclcpp_action::create_server<museum_docent_robot::action::MoveToWaypoint>(
            this, "move_to_waypoint",
            std::bind(&Movement::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Movement::CancelCallback, this, std::placeholders::_1),
            std::bind(&Movement::AcceptedCallback, this, std::placeholders::_1));
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

private:
    // 원본 YAML 기반 Waypoint 변환 로직 유지
    bool LoadWaypointPose(const string& waypoint, geometry_msgs::msg::PoseStamped& target_pose) {
        try {
            YAML::Node config = YAML::LoadFile(package_path + "/config/waypoints.yaml");
            auto artwork = config["waypoints"][waypoint];
            if (!artwork || !artwork["x"] || !artwork["y"] || !artwork["yaw"]) {
                RCLCPP_ERROR(get_logger(), "Invalid waypoint: %s", waypoint.c_str());
                return false;
            }
            const double x = artwork["x"].as<double>();
            const double y = artwork["y"].as<double>();
            const double yaw = artwork["yaw"].as<double>();
            target_pose.header.frame_id = "map";
            target_pose.header.stamp = now();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();
            return true;
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "YAML Error: %s", e.what());
            return false;
        }
    }

    // Goal 수락 시점부터 예약하여 동시 MoveToWaypoint Goal 차단
    rclcpp_action::GoalResponse GoalCallback(
        const rclcpp_action::GoalUUID&,
        shared_ptr<const museum_docent_robot::action::MoveToWaypoint::Goal> goal
    ) {
        if (goal->waypoint != "Start" && goal->waypoint != "Artwork1" &&
            goal->waypoint != "Artwork2" && goal->waypoint != "Artwork3" &&
            goal->waypoint != "Goal") {
            RCLCPP_WARN(get_logger(), "Invalid waypoint: %s", goal->waypoint.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        lock_guard<mutex> lock(goal_mutex_);
        if (phase_ != MovePhase::Idle) {
            RCLCPP_WARN(get_logger(), "MoveToWaypoint busy: rejecting %s", goal->waypoint.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        phase_ = MovePhase::Reserved;
        RCLCPP_INFO(get_logger(), "Goal received: %s", goal->waypoint.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 취소 요청 시 현재 Nav2 Goal 취소 또는 수락 대기 표시
    rclcpp_action::CancelResponse CancelCallback(
        const shared_ptr<GoalHandleMoveToWaypoint> goal_handle
    ) {
        GoalHandleNav2::SharedPtr nav_goal;
        {
            lock_guard<mutex> lock(goal_mutex_);
            if (current_.move_goal != goal_handle || phase_ == MovePhase::Idle) {
                return rclcpp_action::CancelResponse::REJECT;
            }
            phase_ = MovePhase::Canceling;
            nav_goal = current_.nav2_goal;
        }
        if (nav_goal) {
            CancelNav2Goal(nav_goal);
        }
        RCLCPP_INFO(get_logger(), "Cancel requested: %s", goal_handle->get_goal()->waypoint.c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 수락된 Goal을 현재 문맥에 저장하고 기존 Execute Thread 유지
    void AcceptedCallback(shared_ptr<GoalHandleMoveToWaypoint> goal_handle) {
        {
            lock_guard<mutex> lock(goal_mutex_);
            current_.move_goal = goal_handle;
            current_.nav2_goal.reset();
            phase_ = MovePhase::Active;
        }
        thread([this, goal_handle]() { Execute(goal_handle); }).detach();
    }

    // 현재 Nav2 Goal의 취소 요청에서 예외 발생 시 로그 출력
    void CancelNav2Goal(const GoalHandleNav2::SharedPtr& goal_handle) {
        try {
            nav2_client_->async_cancel_goal(goal_handle);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Nav2 cancel failed: %s", e.what());
        }
    }

    // Action 종료 전이가 완료된 뒤에만 다음 Goal 수락 허용
    void CompleteGoal(
        const shared_ptr<GoalHandleMoveToWaypoint>& goal_handle,
        const shared_ptr<museum_docent_robot::action::MoveToWaypoint::Result>& result,
        rclcpp_action::ResultCode nav_code
    ) {
        lock_guard<mutex> lock(goal_mutex_);
        if (current_.move_goal != goal_handle) {
            return;
        }
        try {
            if (goal_handle->is_active()) {
                if (goal_handle->is_canceling() &&
                    (nav_code == rclcpp_action::ResultCode::SUCCEEDED ||
                     nav_code == rclcpp_action::ResultCode::CANCELED)) {
                    result->success = false;
                    result->message = "Canceled";
                    result->reached_waypoint = "";
                    goal_handle->canceled(result);
                } else if (nav_code == rclcpp_action::ResultCode::SUCCEEDED) {
                    goal_handle->succeed(result);
                } else {
                    goal_handle->abort(result);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "MoveToWaypoint terminal transition failed: %s", e.what());
            return;
        }
        current_ = MoveContext{};
        phase_ = MovePhase::Idle;
    }

    // 원본 Nav2 Goal·Feedback·Result 연결을 유지하면서 취소 시점 처리
    void Execute(shared_ptr<GoalHandleMoveToWaypoint> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<museum_docent_robot::action::MoveToWaypoint::Feedback>();
        feedback->status = "Moving to " + goal->waypoint;
        auto result = std::make_shared<museum_docent_robot::action::MoveToWaypoint::Result>();
        geometry_msgs::msg::PoseStamped target_pose;

        if (!LoadWaypointPose(goal->waypoint, target_pose)) {
            result->success = false;
            result->message = "Waypoint not found";
            result->reached_waypoint = "";
            const auto code = goal_handle->is_canceling()
                ? rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::ABORTED;
            CompleteGoal(goal_handle, result, code);
            return;
        }
        if (!nav2_client_->wait_for_action_server(chrono::seconds(5))) {
            result->success = false;
            result->message = "Nav2 server unavailable";
            result->reached_waypoint = "";
            const auto code = goal_handle->is_canceling()
                ? rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::ABORTED;
            CompleteGoal(goal_handle, result, code);
            return;
        }
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Canceled before Nav2 goal";
            result->reached_waypoint = "";
            CompleteGoal(goal_handle, result, rclcpp_action::ResultCode::CANCELED);
            return;
        }

        NavigateToPose::Goal nav2_goal;
        nav2_goal.pose = target_pose;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback =
            [this, goal_handle, result](GoalHandleNav2::SharedPtr nav_goal_handle) {
                if (!nav_goal_handle) {
                    result->success = false;
                    result->message = "Nav2 goal rejected";
                    result->reached_waypoint = "";
                    const auto code = goal_handle->is_canceling()
                        ? rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::ABORTED;
                    CompleteGoal(goal_handle, result, code);
                    return;
                }
                bool request_cancel = false;
                bool current_goal = false;
                {
                    lock_guard<mutex> lock(goal_mutex_);
                    current_goal = current_.move_goal == goal_handle;
                    if (current_goal) {
                        current_.nav2_goal = nav_goal_handle;
                        request_cancel = phase_ == MovePhase::Canceling;
                    }
                }
                if (!current_goal || request_cancel) {
                    CancelNav2Goal(nav_goal_handle);
                }
            };

        options.feedback_callback =
            [goal_handle, feedback](
                GoalHandleNav2::SharedPtr,
                const shared_ptr<const NavigateToPose::Feedback> nav_feedback
            ) {
                if (!goal_handle->is_active()) {
                    return;
                }
                feedback->status = "Distance remaining: " +
                                   to_string(nav_feedback->distance_remaining);
                goal_handle->publish_feedback(feedback);
            };

        options.result_callback =
            [this, goal_handle, result, waypoint = goal->waypoint](
                const GoalHandleNav2::WrappedResult& nav_result
            ) {
                if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    result->success = true;
                    result->message = "Success";
                    result->reached_waypoint = waypoint;
                } else if (nav_result.code == rclcpp_action::ResultCode::CANCELED) {
                    result->success = false;
                    result->message = "Canceled";
                    result->reached_waypoint = "";
                } else {
                    result->success = false;
                    result->message = "Nav2 navigation failed";
                    result->reached_waypoint = "";
                }
                CompleteGoal(goal_handle, result, nav_result.code);
            };

        try {
            nav2_client_->async_send_goal(nav2_goal, options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Nav2 goal send failed: %s", e.what());
            result->success = false;
            result->message = "Nav2 goal send failed";
            result->reached_waypoint = "";
            CompleteGoal(goal_handle, result, rclcpp_action::ResultCode::ABORTED);
        }
    }
};

// 기존 Movement 노드 실행 구조 유지
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto robot_movement = make_shared<Movement>();
    rclcpp::spin(robot_movement);
    rclcpp::shutdown();
    return 0;
}
