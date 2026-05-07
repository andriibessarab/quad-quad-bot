cmake_minimum_required(VERSION 3.8)
project(bot_description)

# Find the build tool for ROS 2
find_package(ament_cmake REQUIRED)

# Install robot description assets and preview launch scripts
install(DIRECTORY urdf meshes launch rviz config DESTINATION share/${PROJECT_NAME})

ament_package()
