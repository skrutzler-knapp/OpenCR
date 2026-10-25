cmake_minimum_required(VERSION 3.10)
project(opencr_command_frame_tests LANGUAGES CXX)

enable_testing()

add_executable(command_frame_test command_frame_test.cpp)
target_compile_features(command_frame_test PRIVATE cxx_std_11)
target_include_directories(command_frame_test PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/../arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/include"
)
target_compile_options(command_frame_test PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME command_frame_test COMMAND command_frame_test)
