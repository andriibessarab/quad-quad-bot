cmake_minimum_required(VERSION 3.8)
project(bot_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(limb_commander src/limb_commander.cpp)

ament_target_dependencies(limb_commander rclcpp rclcpp_action control_msgs)

install(TARGETS limb_commander DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
