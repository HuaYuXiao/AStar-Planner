cmake_minimum_required(VERSION 3.26)
project(Design_of_UAV_dynamic_obstacle_avoidance_algorithm)

set(CMAKE_CXX_STANDARD 14)

include_directories(code/Astar)

add_executable(Design_of_UAV_dynamic_obstacle_avoidance_algorithm
        code/Astar/Astar.cpp
        code/Astar/Astar.h
        code/Astar/main.cpp)
