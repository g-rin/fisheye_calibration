cmake_minimum_required(VERSION 2.8.11)
project(FISHEYE_CALIBRATION)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(fisheye_calibration main.cpp)
target_link_libraries(fisheye_calibration ${OpenCV_LIBS} "-lpopt")
