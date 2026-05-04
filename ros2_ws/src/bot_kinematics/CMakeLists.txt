cmake_minimum_required(VERSION 3.8)
project(bot_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(bot_ik_lib src/ik_solver.cpp)

target_include_directories(
  bot_ik_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bot_ik_lib rclcpp)

ament_export_targets(export_bot_ik_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS bot_ik_lib
  EXPORT export_bot_ik_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

ament_package()
