cmake_minimum_required(VERSION 3.8)
project(bot_sim)

# Find the build tool for ROS 2
find_package(ament_cmake REQUIRED)

# Install the directories containing worlds and sim launch scripts
install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})

ament_package()
