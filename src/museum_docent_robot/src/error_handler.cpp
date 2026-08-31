#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include "museum_docent_robot/srv/set_robot_state.hpp"

#include <memory>
#include <iostream>

using MoveToWaypoint =
    museum_docent_robot::action::MoveToWaypoint;

using GoalHandleMoveToWaypoint =
    rclcpp_action::ClientGoalHandle<MoveToWaypoint>;

using namespace std;

class ErrorHandler : public rclcpp::Node {
    private:
        rclcpp::Client<museum_docent_robot::srv::SetRobotState>::SharedPtr state_client_;
        rclcpp_action::Client<MoveToWaypoint>::SharedPtr action_client_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr test_error_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recovery_service_;
        
    public:
        // 상태 변경 Client·이동 Action Client·Error/Recovery Service 초기화
        ErrorHandler() : rclcpp::Node("error_handler") {
            // StateManager RobotState 변경 요청용 Service Client
            state_client_ =
                create_client<museum_docent_robot::srv::SetRobotState>(
                    "set_robot_state"
                );

            // Recovery 시 Start 이동 요청용 Action Client
            action_client_ =
                rclcpp_action::create_client<MoveToWaypoint>(
                    this,
                    "move_to_waypoint"
                );
            // 테스트용 Error 상태 진입 Service Server
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
            // Error 상태의 Start 복귀 요청용 Recovery Service Server
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
        }
        
        // /test_error true 수신 시 StateManager Error 상태 변경 요청
        void TestErrorCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if(request->data) {
                auto state_request =
                    std::make_shared<museum_docent_robot::srv::SetRobotState::Request>();
                state_request->state = "Error";
                state_client_->async_send_request(state_request);

                response->success = true;
                response->message = "Error triggered";
            }
            else {
                response->success = false;
                response->message = "Error trigger ignored";
            }
        }

        // /recovery_request true 수신 시 Start 복귀 Action Goal 전송
        void RecoveryCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response
        ) {
            if (request->data) {
                MoveToWaypoint::Goal start_goal;
                start_goal.waypoint = "Start";
                rclcpp_action::Client<MoveToWaypoint>::SendGoalOptions send_goal_options;
                // Recovery Action 결과 처리 및 성공 시 Idle 전환
                send_goal_options.result_callback =
                    [this](
                        const GoalHandleMoveToWaypoint::WrappedResult& result
                    ) {
                        switch (result.code) {
                            case rclcpp_action::ResultCode::SUCCEEDED: {
                                auto state_request =
                                    std::make_shared<
                                        museum_docent_robot::srv::SetRobotState::Request
                                    >();

                                state_request->state = "Idle";
                                state_client_->async_send_request(state_request);

                                cout << "Recovery success: Start" << endl;
                                break;
                            }

                            case rclcpp_action::ResultCode::CANCELED: {
                                cout << "Recovery canceled" << endl;
                                break;
                            }

                            case rclcpp_action::ResultCode::ABORTED: {
                                cout << "Recovery failed" << endl;
                                break;
                            }

                            default:
                                break;
                        }
                    };
                                
                action_client_->async_send_goal(
                    start_goal,
                    send_goal_options
                );
                response->success = true;
                response->message = "Recovery started";
            }
            else {
                response->success = false;
                response->message = "Recovery request ignored";
            }
        }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto error_handler =
        std::make_shared<ErrorHandler>();

    rclcpp::spin(error_handler);

    rclcpp::shutdown();

    return 0;
}