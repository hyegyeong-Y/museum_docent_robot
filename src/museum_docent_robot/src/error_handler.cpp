#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include "museum_docent_robot/srv/set_robot_state.hpp"

#include <memory>
#include <iostream>
#include <exception>

using MoveToWaypoint = museum_docent_robot::action::MoveToWaypoint;
using GoalHandleMoveToWaypoint = rclcpp_action::ClientGoalHandle<MoveToWaypoint>;
using namespace std;

class ErrorHandler : public rclcpp::Node {
    private:
        rclcpp::Client<museum_docent_robot::srv::SetRobotState>::SharedPtr state_client_;
        rclcpp_action::Client<MoveToWaypoint>::SharedPtr action_client_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_error_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recovery_service_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_result_publisher_;
        rclcpp::TimerBase::SharedPtr recovery_retry_timer_;

        enum class RecoveryPhase { Idle, Queued, WaitingForResponse, Active, WaitingForState };
        RecoveryPhase recovery_phase_ = RecoveryPhase::Idle;
        bool error_state_seen_ = false;
        int recovery_attempts_ = 0;
        static constexpr int kMaxRecoveryAttempts = 60;

    public:
        // 상태 변경 Client·이동 Action Client·Error/Recovery Service 초기화
        ErrorHandler() : rclcpp::Node("error_handler") {
            state_client_ =
                create_client<museum_docent_robot::srv::SetRobotState>(
                    "set_robot_state"
                );
            recovery_result_publisher_ = create_publisher<std_msgs::msg::String>(
                "recovery_result", 10
            );

            action_client_ =
                rclcpp_action::create_client<MoveToWaypoint>(
                    this,
                    "move_to_waypoint"
                );

            test_error_service_ =
                create_service<std_srvs::srv::SetBool>(
                    "test_error",
                    std::bind(
                        &ErrorHandler::TestErrorCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

            recovery_service_ =
                create_service<std_srvs::srv::SetBool>(
                    "recovery_request",
                    std::bind(
                        &ErrorHandler::RecoveryCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2
                    )
                );

            state_subscription_ = create_subscription<std_msgs::msg::String>(
                "robot_state",
                10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    if (msg->data == "Error") {
                        error_state_seen_ = true;
                    } else if (msg->data == "Idle") {
                        error_state_seen_ = false;
                    }
                }
            );

            recovery_retry_timer_ = create_wall_timer(
                std::chrono::milliseconds(500),
                [this]() {
                    if (recovery_phase_ == RecoveryPhase::Queued) {
                        TrySendRecoveryGoal();
                    }
                }
            );
        }

        // Recovery Goal을 기존 작업 종료 후 수락할 때까지 비동기로 재시도
        void TrySendRecoveryGoal() {
            if (recovery_phase_ != RecoveryPhase::Queued) {
                return;
            }

            if (!error_state_seen_) {
                recovery_phase_ = RecoveryPhase::Idle;
                RCLCPP_WARN(get_logger(), "Recovery canceled: Error state is not active");
                return;
            }

            if (++recovery_attempts_ > kMaxRecoveryAttempts) {
                recovery_phase_ = RecoveryPhase::Idle;
                RCLCPP_ERROR(get_logger(), "Recovery goal could not start: retry limit reached");
                return;
            }

            if (!action_client_->action_server_is_ready()) {
                return;
            }

            recovery_phase_ = RecoveryPhase::WaitingForResponse;
            MoveToWaypoint::Goal start_goal;
            start_goal.waypoint = "Start";
            rclcpp_action::Client<MoveToWaypoint>::SendGoalOptions send_goal_options;

            send_goal_options.goal_response_callback =
                [this](GoalHandleMoveToWaypoint::SharedPtr goal_handle) {
                    if (recovery_phase_ != RecoveryPhase::WaitingForResponse) {
                        return;
                    }

                    if (!goal_handle) {
                        recovery_phase_ = RecoveryPhase::Queued;
                        RCLCPP_WARN(get_logger(), "Recovery goal rejected: retrying");
                        return;
                    }

                    recovery_phase_ = RecoveryPhase::Active;
                    RCLCPP_INFO(get_logger(), "Recovery Start goal accepted");
                };

            send_goal_options.result_callback =
                [this](const GoalHandleMoveToWaypoint::WrappedResult& result) {
                    if (recovery_phase_ != RecoveryPhase::Active &&
                        recovery_phase_ != RecoveryPhase::WaitingForResponse) {
                        return;
                    }

                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        recovery_phase_ = RecoveryPhase::WaitingForState;
                        auto state_request =
                            std::make_shared<museum_docent_robot::srv::SetRobotState::Request>();
                        state_request->state = "Idle";
                        try {
                            state_client_->async_send_request(
                                state_request,
                                [this](rclcpp::Client<museum_docent_robot::srv::SetRobotState>::SharedFuture future) {
                                    try {
                                        auto response = future.get();
                                        if (response->success) {
                                            std_msgs::msg::String msg;
                                            msg.data = "SUCCEEDED";
                                            recovery_result_publisher_->publish(msg);
                                            error_state_seen_ = false;
                                            cout << "Recovery success: Start" << endl;
                                        } else {
                                            RCLCPP_ERROR(get_logger(), "Recovery Idle state update rejected");
                                        }
                                    } catch (const std::exception& e) {
                                        RCLCPP_ERROR(get_logger(), "Recovery Idle response failed: %s", e.what());
                                    }
                                    recovery_phase_ = RecoveryPhase::Idle;
                                }
                            );
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(get_logger(), "Recovery Idle request failed: %s", e.what());
                            recovery_phase_ = RecoveryPhase::Idle;
                        }
                        return;
                    }
                    recovery_phase_ = RecoveryPhase::Idle;
                    switch (result.code) {
                        case rclcpp_action::ResultCode::CANCELED:
                            cout << "Recovery canceled" << endl;
                            break;
                        case rclcpp_action::ResultCode::ABORTED:
                            cout << "Recovery failed" << endl;
                            break;
                        default:
                            RCLCPP_ERROR(get_logger(), "Recovery finished with unknown result");
                            break;
                    }
                };

            try {
                action_client_->async_send_goal(start_goal, send_goal_options);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Recovery send failed: %s", e.what());
                recovery_phase_ = RecoveryPhase::Queued;
            }
        }

        // /test_error true 수신 시 StateManager Error 상태 변경 요청
        void TestErrorCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if (request->data && recovery_phase_ == RecoveryPhase::Idle) {
                auto state_request =
                    std::make_shared<museum_docent_robot::srv::SetRobotState::Request>();
                state_request->state = "Error";
                state_client_->async_send_request(state_request);
                response->success = true;
                response->message = "Error triggered";
            } else {
                response->success = false;
                response->message = "Error trigger ignored";
            }
        }

        // Error 상태에서 한 번만 Recovery 요청을 수락하고 Goal 재시도를 시작
        void RecoveryCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if (!request->data || !error_state_seen_ || recovery_phase_ != RecoveryPhase::Idle) {
                response->success = false;
                response->message = "Recovery request rejected";
                return;
            }

            recovery_attempts_ = 0;
            recovery_phase_ = RecoveryPhase::Queued;
            response->success = true;
            response->message = "Recovery queued";
        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto error_handler = std::make_shared<ErrorHandler>();
    rclcpp::spin(error_handler);
    rclcpp::shutdown();
    return 0;
}
