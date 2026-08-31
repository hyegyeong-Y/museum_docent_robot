#include <rclcpp/rclcpp.hpp>
#include "museum_docent_robot/srv/artwork_description.hpp"
#include <memory>
#include <string>
#include <iostream>
#include <map>

using ArtworkDescription =
museum_docent_robot::srv::ArtworkDescription;
using namespace std;

// 작품 ID-설명 문자열 매핑 관리
const map<string, string> artwork_descriptions = {
    {"Artwork1", "Description for Artwork1"},
    {"Artwork2", "Description for Artwork2"},
    {"Artwork3", "Description for Artwork3"}
};

class ArtworkInfo : public rclcpp::Node {
    private:
        rclcpp::Service<ArtworkDescription>::SharedPtr artwork_description_;

    public:
        // 작품 설명 요청 처리용 Service Server 생성
        ArtworkInfo() : rclcpp::Node("artwork_info") {
            artwork_description_ = create_service<ArtworkDescription>(
                "artwork_description",
                bind(
                    &ArtworkInfo::ArtworkCallback,
                    this,
                    placeholders::_1,
                    placeholders::_2
                )
            );
        };
        // 작품 ID 조회 및 설명·성공 여부 응답
        void ArtworkCallback(
            const std::shared_ptr<ArtworkDescription::Request> request,
            std::shared_ptr<ArtworkDescription::Response> response
        ) {
            auto artwork = artwork_descriptions.find(request->artwork_id);
            if (artwork != artwork_descriptions.end()) {
                cout << "Description: " << artwork->second << endl;
                response->description = artwork->second;
                response->success = true;

            }
            else {
                cout << "Artwork description not found: " << request->artwork_id << endl;
                response->description = "";
                response->success = false;
            }

        }
};

// ArtworkInfo 노드 실행 및 ROS2 이벤트 처리
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto main_info = std::make_shared<ArtworkInfo>();
    rclcpp::spin(main_info);
    rclcpp::shutdown();
    return 0;
}