cmake_minimum_required(VERSION 3.8)
project(bot_command)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(bot_math REQUIRED)

add_executable(locomotion_commander src/locomotion_commander.cpp)

ament_target_dependencies(locomotion_commander rclcpp std_msgs geometry_msgs
                          bot_math)

install(TARGETS locomotion_commander DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
