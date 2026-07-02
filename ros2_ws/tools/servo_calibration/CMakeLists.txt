cmake_minimum_required(VERSION 3.8)
project(servo_calibration)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(../../src/bot_hardware/external/SCServo_Linux SCServo_build)

add_executable(read_pos read_pos.cpp)
target_link_libraries(read_pos SCServo)
