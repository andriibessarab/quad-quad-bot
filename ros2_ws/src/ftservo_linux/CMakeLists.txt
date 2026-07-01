cmake_minimum_required(VERSION 3.8)
project(ftservo_linux)

find_package(ament_cmake REQUIRED)

# Matches the SDK's build: static lib named SCServo, all .h/.cpp in root
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB hdrs *.h)
file(GLOB srs  *.cpp)

add_library(SCServo STATIC ${hdrs} ${srs})

target_include_directories(SCServo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/ftservo_linux>
)

set_target_properties(SCServo PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS SCServo
  EXPORT export_ftservo_linux
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES ${hdrs}
  DESTINATION include/ftservo_linux
)

ament_export_targets(export_ftservo_linux HAS_LIBRARY_TARGET)
ament_export_include_directories(include/ftservo_linux)

ament_package()
