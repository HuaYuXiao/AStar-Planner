cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		sensor_msgs
		geometry_msgs
		nav_msgs
		pcl_ros
		visualization_msgs
		easondrone_msgs
)

catkin_package(
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
)

include_directories(
		include
		SYSTEM
		${catkin_INCLUDE_DIRS}
		${Eigen3_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
		$ENV{HOME}/EasonDrone/Modules/common/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(occupy_map src/occupy_map.cpp)
target_link_libraries(occupy_map ${PCL_LIBRARIES})

add_library(Astar src/Astar.cpp)
target_link_libraries(Astar occupy_map)

add_library(global_planner src/global_planner.cpp)
target_link_libraries(global_planner Astar)

add_executable(global_planner_main src/global_planner_node.cpp)
add_dependencies(global_planner_main ${PROJECT_NAME}_gencpp)
target_link_libraries(global_planner_main
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OCTOMAP_LIBRARIES}
		global_planner
)
