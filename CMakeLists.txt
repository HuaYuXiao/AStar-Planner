cmake_minimum_required(VERSION 3.26.4)
project(Design_of_UAV_dynamic_obstacle_avoidance_algorithm)

set(CMAKE_CXX_STANDARD 14)

include_directories(code/APF_2D_cpp)

add_executable(APF_3D
        code/APF_3D_cpp/APF_3D.cpp
)

find_package(OpenCV REQUIRED COMPONENTS core highgui viz)

# 包含OpenCV头文件路径
include_directories(
        /opt/homebrew/Cellar/opencv/4.8.1_4/include/opencv4/opencv2
        /opt/homebrew/Cellar/vtk/9.2.6_5/include/vtk-9.2
)

# 添加额外的库文件路径
link_directories(
        /opt/homebrew/Cellar/opencv/4.8.1_4/lib
        /opt/homebrew/Cellar/vtk/9.2.6_5/lib
)

target_link_libraries(APF_3D
        ${OpenCV_LIBS}
#        vtkCommonCore vtkCommonDataModel vtkRenderingCore vtkRenderingOpenGL2
)
