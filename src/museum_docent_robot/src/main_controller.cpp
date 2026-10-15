#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include "museum_docent_robot/srv/set_robot_state.hpp"
#include "museum_docent_robot/srv/guide_response.hpp"
#include "museum_docent_robot/srv/artwork_description.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <functional>
#include <iostream>
#include <optional>
#include <cstdint>
#include <exception>

using namespace std;
using MoveToWaypoint = museum_docent_robot::action::MoveToWaypoint;
using GoalHandleMoveToWaypoint = rclcpp_action::ClientGoalHandle<MoveToWaypoint>;
using ArtworkDescription = museum_docent_robot::srv::ArtworkDescription;

// 기존 Waypoint 이름과 순찰 순서 유지
enum class Waypoint {
    Start,
    Artwork1,
    Artwork2,
    Artwork3,
    Goal
};

// Waypoint enum을 Action Goal 문자열로 변환
string WaypointToString(Waypoint waypoint) {
    switch (waypoint) {
        case Waypoint::Start: return "Start";
        case Waypoint::Artwork1: return "Artwork1";
        case Waypoint::Artwork2: return "Artwork2";
        case Waypoint::Artwork3: return "Artwork3";
        case Waypoint::Goal: return "Goal";
        default: return "Unknown";
    }
}

// 작품 ID를 기존 Waypoint로 변환
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
    enum class WorkPhase {
        Idle,
        PatrolMoving,
        UserCanceling,
        Waiting,
        GuideMoving,
        Describing,
        StopCanceling,
        StopReturning,
        Error,
        RecoveryFinishing
    };

    struct GoalContext {
        uint64_t id;
        GoalHandleMoveToWaypoint::SharedPtr handle;
    };

    WorkPhase phase_ = WorkPhase::Idle;
    Waypoint current_waypoint_ = Waypoint::Start;
    optional<GoalContext> goal_;
    uint64_t operation_id_ = 0;

    rclcpp_action::Client<MoveToWaypoint>::SharedPtr action_client_;
    rclcpp::Client<museum_docent_robot::srv::SetRobotState>::SharedPtr state_client_;
    rclcpp::Client<ArtworkDescription>::SharedPtr artwork_client_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_patrol_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr user_request_service_;
    rclcpp::Service<museum_docent_robot::srv::GuideResponse>::SharedPtr guide_response_service_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_result_subscription_;

public:
    // 기존 Service, Topic, Action 인터페이스 유지
    MainController() : rclcpp::Node("main_controller") {
        action_client_ = rclcpp_action::create_client<MoveToWaypoint>(this, "move_to_waypoint");
        state_client_ = create_client<museum_docent_robot::srv::SetRobotState>("set_robot_state");
        artwork_client_ = create_client<ArtworkDescription>("artwork_description");

        start_service_ = create_service<std_srvs::srv::SetBool>(
            "start_signal", std::bind(&MainController::StartCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
        stop_patrol_service_ = create_service<std_srvs::srv::SetBool>(
            "stop_patrol", std::bind(&MainController::StopPatrolCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));
        user_request_service_ = create_service<std_srvs::srv::SetBool>(
            "user_request", std::bind(&MainController::UserRequestCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
        guide_response_service_ = create_service<museum_docent_robot::srv::GuideResponse>(
            "guide_response", std::bind(&MainController::GuideResponseCallback, this,
                                        std::placeholders::_1, std::placeholders::_2));
        state_subscription_ = create_subscription<std_msgs::msg::String>(
            "robot_state", 10,
            std::bind(&MainController::RobotStateCallback, this, std::placeholders::_1));
        recovery_result_subscription_ = create_subscription<std_msgs::msg::String>(
            "recovery_result", 10,
            std::bind(&MainController::RecoveryResultCallback, this, std::placeholders::_1));
    }

private:
    // StateManager에 비동기로 상태 변경 요청
    void SetRobotState(const string& state) {
        auto request = std::make_shared<museum_docent_robot::srv::SetRobotState::Request>();
        request->state = state;
        state_client_->async_send_request(request);
    }

    // 기존 Patrol Waypoint 진행 순서 유지
    Waypoint GetNextWaypoint(Waypoint current) const {
        switch (current) {
            case Waypoint::Start: return Waypoint::Artwork1;
            case Waypoint::Artwork1: return Waypoint::Artwork2;
            case Waypoint::Artwork2: return Waypoint::Artwork3;
            case Waypoint::Artwork3: return Waypoint::Goal;
            case Waypoint::Goal: return Waypoint::Start;
            default: return Waypoint::Start;
        }
    }

    // 현재 수락된 MoveToWaypoint Goal만 취소 요청
    void CancelCurrentGoal() {
        if (!goal_ || !goal_->handle) {
            return;
        }
        try {
            action_client_->async_cancel_goal(goal_->handle);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Cancel request failed: %s", e.what());
        }
    }

    // Error 단계로 전환하고 정상 작업과 늦은 설명 응답 무효화
    void EnterError(bool notify_state_manager = true) {
        if (phase_ != WorkPhase::Error) {
            phase_ = WorkPhase::Error;
            ++operation_id_;
            CancelCurrentGoal();
        }
        if (notify_state_manager) {
            SetRobotState("Error");
        }
    }

    // Patrol 시작은 Idle 상태에서만 허용
    void StartCallback(
        const shared_ptr<std_srvs::srv::SetBool::Request> request,
        shared_ptr<std_srvs::srv::SetBool::Response> response
    ) {
        if (!request->data || phase_ != WorkPhase::Idle || goal_) {
            response->success = false;
            response->message = "Start request rejected";
            return;
        }
        current_waypoint_ = Waypoint::Start;
        phase_ = WorkPhase::PatrolMoving;
        SendGoal();
        response->success = true;
        response->message = "Patrol started";
    }

    // Patrol 중 사용자 요청은 Goal 취소 완료 후 Waiting으로 전환
    void UserRequestCallback(
        const shared_ptr<std_srvs::srv::SetBool::Request> request,
        shared_ptr<std_srvs::srv::SetBool::Response> response
    ) {
        if (!request->data || phase_ != WorkPhase::PatrolMoving || !goal_) {
            response->success = false;
            response->message = "User request ignored";
            return;
        }
        phase_ = WorkPhase::UserCanceling;
        CancelCurrentGoal();
        response->success = true;
        response->message = "User request accepted";
    }

    // Waiting 상태에서만 안내 수락 또는 Patrol 재개
    void GuideResponseCallback(
        const shared_ptr<museum_docent_robot::srv::GuideResponse::Request> request,
        shared_ptr<museum_docent_robot::srv::GuideResponse::Response> response
    ) {
        if (phase_ != WorkPhase::Waiting || goal_) {
            response->accepted = false;
            return;
        }
        if (request->guide) {
            Waypoint selected;
            if (!StringToWaypoint(request->artwork_id, selected)) {
                response->accepted = false;
                cout << "wrong Art_id" << endl;
                return;
            }
            current_waypoint_ = selected;
            phase_ = WorkPhase::GuideMoving;
            cout << "Mode changed to: Guide" << endl;
        } else {
            phase_ = WorkPhase::PatrolMoving;
            cout << "Mode changed to: Patrol" << endl;
        }
        response->accepted = true;
        SendGoal();
    }

    // 작품 설명 비동기 응답은 요청 시점의 작업과 일치할 때만 적용
    void RequestArtworkDescription(const string& artwork_id) {
        auto request = std::make_shared<ArtworkDescription::Request>();
        request->artwork_id = artwork_id;
        const uint64_t request_id = ++operation_id_;
        artwork_client_->async_send_request(
            request,
            [this, request_id](rclcpp::Client<ArtworkDescription>::SharedFuture future) {
                if (phase_ != WorkPhase::Describing || operation_id_ != request_id) {
                    return;
                }
                try {
                    auto response = future.get();
                    if (response->success) {
                        cout << "Artwork description: " << response->description << endl;
                    } else {
                        cout << "Artwork description request failed" << endl;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Artwork description error: %s", e.what());
                }
                FinishGuide();
            });
    }

    // 기존 Guide 완료 후 해당 작품 다음 Waypoint부터 Patrol 재개
    void FinishGuide() {
        if (phase_ != WorkPhase::Describing) {
            return;
        }
        cout << "Guide completed: " << WaypointToString(current_waypoint_) << endl;
        cout << "Mode changed to: Patrol" << endl;
        current_waypoint_ = GetNextWaypoint(current_waypoint_);
        phase_ = WorkPhase::PatrolMoving;
        SendGoal();
    }

    // 점검 Stop은 현재 정상 작업을 종료하고 Start 복귀
    void StopPatrolCallback(
        const shared_ptr<std_srvs::srv::SetBool::Request> request,
        shared_ptr<std_srvs::srv::SetBool::Response> response
    ) {
        const bool stoppable = phase_ == WorkPhase::PatrolMoving ||
                               phase_ == WorkPhase::UserCanceling ||
                               phase_ == WorkPhase::Waiting ||
                               phase_ == WorkPhase::GuideMoving ||
                               phase_ == WorkPhase::Describing;
        if (!request->data || !stoppable) {
            response->success = false;
            response->message = "Stop request rejected";
            return;
        }
        phase_ = WorkPhase::StopCanceling;
        ++operation_id_;
        if (goal_) {
            CancelCurrentGoal();
        } else {
            ReturnToStart();
        }
        response->success = true;
        response->message = "Patrol stop requested";
    }

    // 기존 Goal의 결과를 받은 뒤 점검용 Start Goal 전송
    void ReturnToStart() {
        if (phase_ != WorkPhase::StopCanceling || goal_) {
            return;
        }
        current_waypoint_ = Waypoint::Start;
        phase_ = WorkPhase::StopReturning;
        RCLCPP_INFO(get_logger(), "Returning to Start");
        SendGoal();
    }

    // Recovery 성공 이벤트를 받은 뒤에만 Idle 상태로 작업 단계 복귀
    void RobotStateCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "Error" && phase_ != WorkPhase::RecoveryFinishing) {
            EnterError(false);
        } else if (msg->data == "Idle" && phase_ == WorkPhase::RecoveryFinishing) {
            current_waypoint_ = Waypoint::Start;
            phase_ = WorkPhase::Idle;
            ++operation_id_;
            RCLCPP_INFO(get_logger(), "Recovery complete: controller reset to Idle");
        }
    }

    // ErrorHandler가 보낸 명시적 복구 완료 이벤트만 반영
    void RecoveryResultCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data != "SUCCEEDED" || phase_ != WorkPhase::Error) {
            return;
        }
        goal_.reset();
        ++operation_id_;
        phase_ = WorkPhase::RecoveryFinishing;
    }

    // Goal 요청부터 최종 결과까지 하나의 GoalContext로 식별
    void SendGoal() {
        const bool moving = phase_ == WorkPhase::PatrolMoving ||
                            phase_ == WorkPhase::GuideMoving ||
                            phase_ == WorkPhase::StopReturning;
        if (!moving || goal_) {
            return;
        }

        MoveToWaypoint::Goal request;
        request.waypoint = WaypointToString(current_waypoint_);
        const uint64_t request_id = ++operation_id_;
        goal_.emplace(GoalContext{request_id, nullptr});
        SetRobotState("Moving");
        rclcpp_action::Client<MoveToWaypoint>::SendGoalOptions options;

        options.goal_response_callback =
            [this, request_id](GoalHandleMoveToWaypoint::SharedPtr handle) {
                if (!goal_ || goal_->id != request_id) {
                    if (handle) {
                        try {
                            action_client_->async_cancel_goal(handle);
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(get_logger(), "Late goal cancel failed: %s", e.what());
                        }
                    }
                    return;
                }
                if (!handle) {
                    goal_.reset();
                    RCLCPP_ERROR(get_logger(), "Movement goal rejected");
                    EnterError();
                    return;
                }
                goal_->handle = handle;
                if (phase_ == WorkPhase::UserCanceling ||
                    phase_ == WorkPhase::StopCanceling ||
                    phase_ == WorkPhase::Error) {
                    CancelCurrentGoal();
                }
            };

        options.feedback_callback =
            [this, request_id](
                GoalHandleMoveToWaypoint::SharedPtr,
                const shared_ptr<const MoveToWaypoint::Feedback> feedback
            ) {
                if (goal_ && goal_->id == request_id && phase_ != WorkPhase::Error) {
                    cout << feedback->status << endl;
                }
            };

        options.result_callback =
            [this, request_id](const GoalHandleMoveToWaypoint::WrappedResult& result) {
                if (!goal_ || goal_->id != request_id) {
                    return;
                }
                goal_.reset();

                if (phase_ == WorkPhase::Error) {
                    return;
                }
                if (result.code == rclcpp_action::ResultCode::ABORTED ||
                    result.code == rclcpp_action::ResultCode::UNKNOWN) {
                    EnterError();
                    return;
                }
                if (phase_ == WorkPhase::StopCanceling) {
                    ReturnToStart();
                    return;
                }
                if (phase_ == WorkPhase::StopReturning) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        current_waypoint_ = Waypoint::Start;
                        phase_ = WorkPhase::Idle;
                        SetRobotState("Idle");
                        RCLCPP_INFO(get_logger(), "Patrol stopped at Start");
                    } else {
                        EnterError();
                    }
                    return;
                }
                if (phase_ == WorkPhase::UserCanceling) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED ||
                        result.code == rclcpp_action::ResultCode::CANCELED) {
                        phase_ = WorkPhase::Waiting;
                        SetRobotState("Waiting");
                    } else {
                        EnterError();
                    }
                    return;
                }
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    EnterError();
                    return;
                }

                if (phase_ == WorkPhase::PatrolMoving) {
                    SetRobotState("Arrived");
                    current_waypoint_ = GetNextWaypoint(current_waypoint_);
                    SendGoal();
                } else if (phase_ == WorkPhase::GuideMoving) {
                    SetRobotState("Arrived");
                    phase_ = WorkPhase::Describing;
                    RequestArtworkDescription(WaypointToString(current_waypoint_));
                }
            };

        try {
            action_client_->async_send_goal(request, options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Goal send failed: %s", e.what());
            goal_.reset();
            EnterError();
        }
    }
};

// 기존 MainController ROS2 spin 유지
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto main_controller = make_shared<MainController>();
    rclcpp::spin(main_controller);
    rclcpp::shutdown();
    return 0;
}
