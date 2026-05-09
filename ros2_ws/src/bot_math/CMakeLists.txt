cmake_minimum_required(VERSION 3.8)
project(bot_math)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(bot_math_lib src/ik_solver.cpp src/limb_trajectory.cpp)

target_include_directories(
  bot_math_lib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bot_math_lib geometry_msgs)

ament_export_targets(export_bot_math_lib HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS bot_math_lib
  EXPORT export_bot_math_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

ament_package()
