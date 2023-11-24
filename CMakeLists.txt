cmake_minimum_required(VERSION 3.26)
project(Design_of_UAV_dynamic_obstacle_avoidance_algorithm)

set(CMAKE_CXX_STANDARD 14)

include_directories(code/APF_2D_cpp)

add_executable(Design_of_UAV_dynamic_obstacle_avoidance_algorithm
        code/APF_2D_cpp/ArtificialPotential.h
        code/APF_2D_cpp/ArtificialPotential.cpp)
