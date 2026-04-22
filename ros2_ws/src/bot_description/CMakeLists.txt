cmake_minimum_required(VERSION 3.8)
project(bot_description)

# Find the build tool for ROS 2
find_package(ament_cmake REQUIRED)

# Install the directories containing URDF and meshes
install(
        DIRECTORY urdf meshes
        DESTINATION share/${PROJECT_NAME}
)

ament_package()