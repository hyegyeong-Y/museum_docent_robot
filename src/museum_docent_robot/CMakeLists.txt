cmake_minimum_required(VERSION 3.8)
project(museum_docent_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetRobotState.srv"
  "srv/GuideResponse.srv"
  "srv/ArtworkDescription.srv"
  "action/MoveToWaypoint.action"
)

add_executable(state_manager src/state_manager.cpp)
add_executable(movement src/movement.cpp)
add_executable(main_controller src/main_controller.cpp)
add_executable(artwork_info src/artwork_info.cpp)
add_executable(error_handler src/error_handler.cpp)

ament_target_dependencies(state_manager
  rclcpp
  std_msgs
)

ament_target_dependencies(movement
  rclcpp
  rclcpp_action
)

ament_target_dependencies(main_controller
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
)

ament_target_dependencies(artwork_info
  rclcpp
  std_srvs
)

ament_target_dependencies(error_handler
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
)

ament_target_dependencies(movement
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(state_manager
  "${cpp_typesupport_target}"
)
target_link_libraries(movement
  "${cpp_typesupport_target}"
  yaml-cpp
)
target_link_libraries(main_controller
  "${cpp_typesupport_target}"
)
target_link_libraries(artwork_info
  "${cpp_typesupport_target}"
)
target_link_libraries(error_handler
  "${cpp_typesupport_target}"
)

install(TARGETS
  state_manager
  movement
  main_controller
  artwork_info
  error_handler
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()