cmake_minimum_required(VERSION 3.8)
project(bot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()
