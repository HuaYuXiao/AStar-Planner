cmake_minimum_required(VERSION 3.26.4)
project(UAV_dynamic_obstacle_avoidance)

set(CMAKE_CXX_STANDARD 14)

include_directories(
        scripts/APF_2D_point_cpp
)

add_executable(readT265
        source/readT265.cpp
)
add_executable(APF_3D_point
        scripts/APF_3D_point_cpp/APF_3D.cpp
)

find_package(OpenCV REQUIRED COMPONENTS core highgui viz)
find_package(realsense2 REQUIRED)


# 包含OpenCV头文件路径
include_directories(
        /opt/homebrew/Cellar/opencv/4.8.1_4/include/opencv4/opencv2
)

# 添加额外的库文件路径
link_directories(
        /opt/homebrew/Cellar/opencv/4.8.1_4/lib
)

target_link_libraries(readT265
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
)

