cmake_minimum_required(VERSION 2.8.4)
project(kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("$ENV{EIGEN3_INCLUDE_DIR}")

set(SOURCE_FILES kalman.cpp example.cpp)
add_executable(example ${SOURCE_FILES})
