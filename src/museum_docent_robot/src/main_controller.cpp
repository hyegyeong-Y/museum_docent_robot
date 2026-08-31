#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include "museum_docent_robot/srv/set_robot_state.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <std_srvs/srv/set_bool.hpp>
#include "museum_docent_robot/srv/guide_response.hpp"
#include "museum_docent_robot/srv/artwork_description.hpp"
#include <std_msgs/msg/string.hpp>

using MoveToWaypoint = museum_docent_robot::action::MoveToWaypoint;

using GoalHandleMoveToWaypoint =
    rclcpp_action::ClientGoalHandle<MoveToWaypoint>;

using ArtworkDescription =
    museum_docent_robot::srv::ArtworkDescription;

using namespace std;

// 순찰·안내용 주요 이동 지점 정의
enum class Waypoint {
    Start,
    Artwork1,
    Artwork2,
    Artwork3,
    Goal
};

// Waypoint enum → ROS2 메시지용 문자열 변환
string WaypointToString(Waypoint waypoint) {
    switch (waypoint) {
        case Waypoint::Start:
            return "Start";

        case Waypoint::Artwork1:
            return "Artwork1";

        case Waypoint::Artwork2:
            return "Artwork2";

        case Waypoint::Artwork3:
            return "Artwork3";

        case Waypoint::Goal:
            return "Goal";

        default:
            return "Unknown";
    }
}

// 로봇 동작 모드 Patrol·Guide 구분
enum class RobotMode {
    Patrol,
    Guide
};

// RobotMode → 로그 출력용 문자열 변환
string RobotModeToString(RobotMode mode) {
    switch (mode) {
        case RobotMode::Patrol:
            return "Patrol";

        case RobotMode::Guide:
            return "Guide";

        default:
            return "Unknown";
    }
}

// 작품 ID 문자열 → Waypoint enum 변환
bool StringToWaypoint(const string& artwork_id, Waypoint& waypoint) {
    if (artwork_id == "Artwork1") {
        waypoint = Waypoint::Artwork1;
        return true;
    }

    if (artwork_id == "Artwork2") {
        waypoint = Waypoint::Artwork2;
        return true;
    }

    if (artwork_id == "Artwork3") {
        waypoint = Waypoint::Artwork3;
        return true;
    }

    return false;
}

class MainController : public rclcpp::Node {
    private:
        Waypoint current_waypoint_;
        RobotMode robot_mode_;
        bool error_active_ = false;
        rclcpp_action::Client<MoveToWaypoint>::SharedPtr action_client_;
        rclcpp::Client<museum_docent_robot::srv::SetRobotState>::SharedPtr state_client_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
        GoalHandleMoveToWaypoint::SharedPtr active_goal_handle_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr user_request_service_;
        rclcpp::Service<museum_docent_robot::srv::GuideResponse>::SharedPtr guide_response_service_;
        rclcpp::Client<ArtworkDescription>::SharedPtr artwork_client_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;

    public:
        // MainController Action Client·Service Client/Server·Subscriber 초기화
        MainController():rclcpp::Node("main_controller"){
            current_waypoint_ = Waypoint::Start;
            robot_mode_ = RobotMode::Patrol;
            action_client_ =
                rclcpp_action::create_client<MoveToWaypoint>(
                    this,
                    "move_to_waypoint"
                );

            state_client_ =
                create_client<museum_docent_robot::srv::SetRobotState>(
                    "set_robot_state"
                );

            // Start patrol service
            start_service_ =
                create_service<std_srvs::srv::SetBool>(
                    "start_signal",
                    std::bind(
                        &MainController::StartCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

            // User request service
            user_request_service_ =
                create_service<std_srvs::srv::SetBool>(
                    "user_request",
                    std::bind(
                        &MainController::UserRequestCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

               guide_response_service_ =
                create_service<museum_docent_robot::srv::GuideResponse>(
                    "guide_response",
                    std::bind(
                        &MainController::GuideResponseCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );
                artwork_client_ =
                    create_client<ArtworkDescription>(
                        "artwork_description"
                    );
                state_subscription_ =
                    create_subscription<std_msgs::msg::String>(
                        "robot_state",
                        10,
                        std::bind(
                            &MainController::RobotStateCallback,
                            this,
                            std::placeholders::_1
                        )
                    );
        }
        // StateManager RobotState 비동기 변경 요청
        void SetRobotState(const string& state) {
            auto request =
                std::make_shared<museum_docent_robot::srv::SetRobotState::Request>();

            request->state = state;
            state_client_->async_send_request(request);
        }

        // 현재 순찰 지점 기준 다음 Waypoint 결정
        Waypoint GetNextWaypoint(Waypoint current) {
            switch(current) {
                case Waypoint::Start:
                    return Waypoint::Artwork1;

                case Waypoint::Artwork1:
                    return Waypoint::Artwork2;

                case Waypoint::Artwork2:
                    return Waypoint::Artwork3;

                case Waypoint::Artwork3:
                    return Waypoint::Goal;

                case Waypoint::Goal:
                    return Waypoint::Start;
                    
                return Waypoint::Start;
            }
        }

        // /start_signal 수신 및 Patrol 이동 시작
        void StartCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if (request->data) {
                SendGoal();

                response->success = true;
                response->message = "Patrol started";
            }
            else {
                response->success = false;
                response->message = "Start request rejected";
            }
        }

        // 사용자 안내 요청 시 현재 이동 취소 및 Waiting 전환
        void UserRequestCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if (request->data) {
                if (active_goal_handle_) {
                    action_client_->async_cancel_goal(active_goal_handle_);
                }

                SetRobotState("Waiting");

                response->success = true;
                response->message = "User request accepted";
            }
            else {
                response->success = false;
                response->message = "User request ignored";
            }
        }

        // 안내 수락 여부에 따른 Guide 이동·Patrol 복귀 결정
        void GuideResponseCallback(
            const std::shared_ptr<museum_docent_robot::srv::GuideResponse::Request> request,
            std::shared_ptr<museum_docent_robot::srv::GuideResponse::Response> response
        ) {
            if (request->guide) {
                if (StringToWaypoint(request->artwork_id, current_waypoint_)) {
                    response->accepted = true;
                    robot_mode_ = RobotMode::Guide;

                    cout << "Mode changed to: "
                        << RobotModeToString(robot_mode_) << endl;

                    SendGoal();
                }
                else {
                    response->accepted = false;
                    cout << "wrong Art_id" << endl;
                }
            }
            else {
                response->accepted = true;
                robot_mode_ = RobotMode::Patrol;
                SendGoal();
            }
        }
        // Guide 도착 후 ArtworkInfo 작품 설명 비동기 요청
        void RequestArtworkDescription(const string& artwork_id) {
            auto request =
                std::make_shared<ArtworkDescription::Request>();

            request->artwork_id = artwork_id;

            artwork_client_->async_send_request(
                request,
                [this](
                    rclcpp::Client<ArtworkDescription>::SharedFuture future
                ) {
                    auto response = future.get();

                    if (response->success) {
                        cout << "Artwork description: "
                            << response->description << endl;
                    }
                    else {
                        cout << "Artwork description request failed" << endl;
                    }
                    FinishGuide();
                }
            );
        }

        // 작품 설명 처리 후 Guide 종료 및 다음 Patrol 지점 복귀
        void FinishGuide() {
            cout << "Guide completed: "
                << WaypointToString(current_waypoint_) << endl;

            robot_mode_ = RobotMode::Patrol;

            cout << "Mode changed to: "
                << RobotModeToString(robot_mode_) << endl;

            current_waypoint_ = GetNextWaypoint(current_waypoint_);
            SendGoal();
        }

        // /robot_state 구독 및 Error 진입 시 기존 Goal 1회 취소
        void RobotStateCallback(
            const std_msgs::msg::String::SharedPtr msg
        ) {
            if (msg->data == "Error" && !error_active_) {
                error_active_ = true;

                action_client_->async_cancel_all_goals();
            }
            else if (msg->data == "Idle") {
                error_active_ = false;
            }
        }

        // 현재 Waypoint Action Goal 전송 및 콜백 처리
        void SendGoal() {
            MoveToWaypoint::Goal goal_msg;
            goal_msg.waypoint = WaypointToString(current_waypoint_);

            rclcpp_action::Client<MoveToWaypoint>::SendGoalOptions send_goal;

            // Goal 수락 시 현재 활성 GoalHandle 저장
            send_goal.goal_response_callback =
                [this](GoalHandleMoveToWaypoint::SharedPtr goal_handle) {
                    if (goal_handle) {
                        active_goal_handle_ = goal_handle;
                    }
                };

            // 이동 중 Feedback 상태 문자열 출력
            send_goal.feedback_callback =
                [this](
                    GoalHandleMoveToWaypoint::SharedPtr goal_handle,
                    const shared_ptr<const MoveToWaypoint::Feedback> feedback
                ) {
                    cout << feedback->status << endl;
                };

            // Action 종료 상태별 Patrol/Guide·Waiting·Error 전이 처리
            send_goal.result_callback =
                [this](
                    const GoalHandleMoveToWaypoint::WrappedResult& result
                ) {
                    switch (result.code) {
                        case rclcpp_action::ResultCode::SUCCEEDED: {
                            if (error_active_) {
                                break;
                            }
                            SetRobotState("Arrived");

                            if (robot_mode_ == RobotMode::Patrol) {
                                current_waypoint_ = GetNextWaypoint(current_waypoint_);
                                SendGoal();
                            }
                            else if (robot_mode_ == RobotMode::Guide) {

                                RequestArtworkDescription(
                                    WaypointToString(current_waypoint_)
                                );
                            }
                            break;
                        }

                        case rclcpp_action::ResultCode::CANCELED:
                            if (!error_active_) {
                                SetRobotState("Waiting");
                            }
                            break;

                        case rclcpp_action::ResultCode::ABORTED: {
                            SetRobotState("Error");           
                            break;
                        }

                        default:
                            break;
                    }
                };

            action_client_->async_send_goal(
                goal_msg,
                send_goal
            );
        }
};

// MainController 노드 실행 및 전체 Patrol/Guide 흐름 관리
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto main_controller =
        std::make_shared<MainController>();

    rclcpp::spin(main_controller);

    rclcpp::shutdown();

    return 0;
}