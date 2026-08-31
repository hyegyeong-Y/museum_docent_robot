#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "museum_docent_robot/action/move_to_waypoint.hpp"
#include <thread>
#include <functional>

using namespace std;
using GoalHandleMoveToWaypoint =
    rclcpp_action::ServerGoalHandle<museum_docent_robot::action::MoveToWaypoint>;

class Movement : public rclcpp::Node{
    private:
        rclcpp_action::Server<museum_docent_robot::action::MoveToWaypoint>::SharedPtr action_server_;

    public:
        // Goal·Cancel·Accepted 콜백 연결 Action Server 생성
        Movement():rclcpp::Node("movement") {
            action_server_ = rclcpp_action::create_server<museum_docent_robot::action::MoveToWaypoint>(
                this,
                "move_to_waypoint",
                std::bind(
                    &Movement::GoalCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                ),
                std::bind(
                    &Movement::CancelCallback,
                    this,
                    std::placeholders::_1
                ),
                std::bind(
                    &Movement::AcceptedCallback,
                    this,
                    std::placeholders::_1
                )
            );
        }
        // 요청 Waypoint 유효성 검사 및 Goal 수락·거절 결정
        rclcpp_action::GoalResponse GoalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const museum_docent_robot::action::MoveToWaypoint::Goal> goal
            ) 
            {
                if(goal->waypoint == "Start" || goal->waypoint == "Artwork1" || goal->waypoint == "Artwork2" ||
                    goal->waypoint == "Artwork3" || goal->waypoint == "Goal") {
                    cout << "Goal received: " << goal->waypoint << endl;
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
                else {
                    cout << "Invalid waypoint: " << goal->waypoint << endl;
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }

        // 클라이언트 Action 취소 요청 허용
        rclcpp_action::CancelResponse CancelCallback(
                const std::shared_ptr<
                    rclcpp_action::ServerGoalHandle<museum_docent_robot::action::MoveToWaypoint>> goal_handle
            )
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            }

        // 수락 Goal의 Execute 별도 Thread 실행
        void AcceptedCallback(shared_ptr<GoalHandleMoveToWaypoint> goal_handle) {
                std::thread(
                    [this, goal_handle]() {
                        Execute(goal_handle);
                    }
                ).detach();
            }

        // 이동 모사 및 Feedback·Cancel·Result 처리
        void Execute(std::shared_ptr<GoalHandleMoveToWaypoint> goal_handle) {
            auto goal = goal_handle->get_goal();

            auto feedback =
                std::make_shared<museum_docent_robot::action::MoveToWaypoint::Feedback>();

            feedback->status = "Moving to " + goal->waypoint;
            
            auto result = std::make_shared<museum_docent_robot::action::MoveToWaypoint::Result>();

            // 테스트용 특정 지점 Action ABORTED 실패 시뮬레이션
            // if (goal->waypoint == "Artwork3") {
            //     result->success = false;
            //     result->message = "Movement failed";
            //     result->reached_waypoint = "";

            //     goal_handle->abort(result);
            //     return;
            // }

            // 1초 간격 Feedback 발행 및 중간 Cancel 요청 확인
            for (int i=0; i<3; i++) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "Canceled";
                    result->reached_waypoint = "";

                    goal_handle->canceled(result);
                    return;
                }
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
                result->success = true;
                result->message = "Success";
                result->reached_waypoint = goal->waypoint;
                cout << "Movement success: " << goal->waypoint << endl;
                

                // 이동 완료 후 성공 Result 반환 및 SUCCEEDED 종료
                goal_handle->succeed(result);
                return;

        }
};

// Movement Action Server 노드 실행
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto robot_movement = make_shared<Movement>();
    rclcpp::spin(robot_movement);
    rclcpp::shutdown();
    return 0;
}
