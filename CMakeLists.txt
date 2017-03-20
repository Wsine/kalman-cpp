cmake_minimum_required(VERSION 2.8.4)
project(kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB LIB_HEAD_FILES kalman.hpp)
file(GLOB LIB_CPP_FILES kalman.cpp)
set(LIB_SOURCE_FILES ${LIB_HEAD_FILES} ${LIB_CPP_FILES})
add_library(kalman-cpp SHARED ${LIB_SOURCE_FILES})

