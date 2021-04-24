# in order to run cmake with different compiler, 
# use "cmake -D CMAKE_C_COMPILER=gcc-5 -D CMAKE_CXX_COMPILER=g++-5 .."
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# set (CMAKE_CXX_STANDARD 14)

project(playback)

set(CXX_FLAGS "-Wall")
add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} CMAKE_CXX_STANDARD 14")
# set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.7.2 REQUIRED)
message("PCL_LIBRARIES - ${PCL_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})