#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "museum_docent_robot/srv/set_robot_state.hpp"
#include <string>
#include <memory>
#include <chrono>

using namespace std; 

// 로봇 전체 상태 enum 정의
enum class RobotState {
    Idle,
    Moving,
    Waiting,
    Arrived,
    Error
};

// RobotState enum → Topic·로그용 문자열 변환
string RobotStateToString(RobotState state) {
    switch(state) {
        case(RobotState::Idle):
            return "Idle";
        case(RobotState::Moving):
            return "Moving";
        case(RobotState::Waiting):
            return "Waiting";
        case(RobotState::Arrived):
            return "Arrived";
        case(RobotState::Error):
            return "Error";
        default:
            return "None";
    }
        
}

// Service 상태 문자열 → RobotState enum 변환
bool StringToRobotState(const string& state_text, RobotState& state) {
        if (state_text == "Moving") {
            state = RobotState::Moving;
            return true;
        }
        else if (state_text == "Waiting") {
            state = RobotState::Waiting;
            return true;
        }
        else if (state_text == "Arrived") {
            state = RobotState::Arrived;
            return true;
        }
        else if (state_text == "Error") {
            state = RobotState::Error;
            return true;
        }
        else if (state_text == "Idle") {
            state = RobotState::Idle;
            return true;
        }
        else {
            cout << "Wrong string please check" << endl;
            return false;

        }
}

class StateManager : public rclcpp::Node {
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        RobotState robot_state_ = RobotState::Idle;
        rclcpp::Service<museum_docent_robot::srv::SetRobotState>::SharedPtr state_service_;
    public:
        // RobotState Publisher·heartbeat Timer·상태 변경 Service Server 초기화
        StateManager():rclcpp::Node("state_manager"){
            state_publisher_ = create_publisher<std_msgs::msg::String>(
                "robot_state",
                10
            );
            // 현재 RobotState 1초 주기 /robot_state 발행
            timer_ = create_wall_timer(
                chrono::seconds(1), [this]() {
                    std_msgs::msg::String state_msg;
                    state_msg.data = RobotStateToString(robot_state_);
                    state_publisher_ -> publish(state_msg);
                    
                }
            );
            
            // /set_robot_state 요청 기반 상태 변경 및 즉시 /robot_state 반영
            state_service_ = create_service<museum_docent_robot::srv::SetRobotState> (
                "set_robot_state",
                [this](museum_docent_robot::srv::SetRobotState::Request::SharedPtr request, museum_docent_robot::srv::SetRobotState::Response::SharedPtr response) {
                   bool success = StringToRobotState(request->state, robot_state_);
                   if(success) {
                        // 상태 변경 직후 새 상태 즉시 publish
                        std_msgs::msg::String state_msg;
                        state_msg.data = RobotStateToString(robot_state_);
                        state_publisher_->publish(state_msg);

                        response->success = true;

                        cout << "RobotStateChange: "
                            << RobotStateToString(robot_state_)
                            << endl;
                    }
                   else {
                        response->success = false;
                   }
                  
                }
            );
        }

};

// StateManager 노드 실행 및 RobotState 지속 관리
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto robot_state = make_shared<StateManager>();
    rclcpp::spin(robot_state);
    rclcpp::shutdown();
    return 0;
}