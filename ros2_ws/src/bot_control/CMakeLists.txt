cmake_minimum_required(VERSION 3.8)
project(bot_control)

# Find the build tool for ROS 2
find_package(ament_cmake REQUIRED)

# Install the directories
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
