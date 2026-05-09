cmake_minimum_required(VERSION 3.8)
project(bot_gait)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bot_math REQUIRED)

add_executable(limb_trajectory_generator src/limb_trajectory_generator.cpp)

ament_target_dependencies(limb_trajectory_generator rclcpp geometry_msgs
                          bot_math)

install(TARGETS limb_trajectory_generator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
