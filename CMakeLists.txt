cmake_minimum_required(VERSION 3.26.4)
project(Design_of_UAV_dynamic_obstacle_avoidance_algorithm)

set(CMAKE_CXX_STANDARD 14)

include_directories(code/APF_2D_cpp)

add_executable(Design_of_UAV_dynamic_obstacle_avoidance_algorithm
        code/APF_3D_cpp/APF.cpp)

find_package(OpenCV REQUIRED)

# 包含OpenCV头文件路径
include_directories(/opt/homebrew/Cellar/opencv/4.8.1_4/include/opencv4/opencv2)

# 添加额外的库文件路径
link_directories(/opt/homebrew/Cellar/opencv/4.8.1_4/lib)

target_link_libraries(Design_of_UAV_dynamic_obstacle_avoidance_algorithm ${OpenCV_LIBS})

