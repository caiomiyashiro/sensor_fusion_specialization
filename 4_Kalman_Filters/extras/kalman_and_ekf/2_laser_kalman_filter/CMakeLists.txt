cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

project(kalman_2)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Executable for create matrix exercise
add_executable (kalmacaio src/main.cpp src/kalman_filter.cpp src/tracking.cpp)
target_link_libraries (kalmacaio Eigen3::Eigen)