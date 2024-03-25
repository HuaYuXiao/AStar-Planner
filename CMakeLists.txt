cmake_minimum_required(VERSION 2.8.3)
project(uav_navigation)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
		prometheus_msgs
)

catkin_package(
		INCLUDE_DIRS include
		LIBRARIES uav_navigation
)

include_directories(
		SYSTEM
		${catkin_INCLUDE_DIRS}
		${Eigen3_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/include
		~/Prometheus/Modules/common/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(uav_navigation
		src/global_planner_node.cpp
		src/global_planner.cpp
		src/Astar.cpp
		src/occupy_map.cpp
)

target_link_libraries(uav_navigation
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OCTOMAP_LIBRARIES}
)
