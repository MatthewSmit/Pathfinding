cmake_minimum_required(VERSION 3.3)
project(Pathfinding)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp Pathfinding.cpp Fringe.cpp JPS.cpp)
add_executable(Pathfinding ${SOURCE_FILES})