cmake_minimum_required(VERSION 3.8)
project(bot_hardware)

# require cpp 17+ for certain features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- Vendored Feetech SDK ----------------------------------------
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(external/SCServo_Linux)

# --- hardware interface plugin ----------------------------------
add_library(${PROJECT_NAME} SHARED src/sts_system_interface.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} SCServo)

ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp
                          rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

ament_export_targets(export_bot_hardware HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_bot_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

ament_package()
