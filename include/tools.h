#ifndef _TOOLS_H
#define _TOOLS_H

/*
```#ifndef``` 是 C/C++ 中的预处理器指令，用于防止头文件的多重包含（multiple inclusion）。当一个头文件被包含到多个源文件中时，如果没有适当的保护，就可能会导致重复定义的问题。

以下是 `#ifndef` 的工作原理：

1. 当第一次包含头文件时，`TOOLS_H` 这个宏会被设置为未定义（undefined）状态。
2. 随后的包含操作会检查 `TOOLS_H` 这个宏是否已经被定义。如果已经定义了，就意味着这个头文件已经被包含过了，就不需要再次包含了。
3. 如果 `TOOLS_H` 这个宏没有被定义，就会执行 `#define TOOLS_H` 指令将其定义为一个非零值。
4. 在文件的末尾，会有 `#endif` 指令，用于结束 `#ifndef` 块。

这样做的好处是，在多个源文件中包含同一个头文件时，预处理器只会将头文件包含一次，从而避免了重复定义的问题，提高了编译效率。

所以，`#ifndef TOOLS_H` 的作用是检查 `TOOLS_H` 这个宏是否已经被定义，如果没有被定义，则说明这是第一次包含 `tools.h`，可以执行相应的操作。
 */

#include <iostream>
#include <algorithm>
#include <queue>
#include <string>
#include <unordered_map>
#include <sstream>

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"
#include <visualization_msgs/Marker.h>
#include "message_utils.h"


// 是否2D规划
extern bool is_2D;
// 参数
extern double fly_height_2D;
extern double safe_distance;
extern double time_per_path;
extern int map_input;
extern double replan_time;
extern bool sim_mode;
extern bool map_groundtruth;
// 启发式参数
extern double lambda_heu_;
// 最大搜索次数
extern int max_search_num;
// 地图分辨率
extern double resolution_;
// 膨胀参数
extern double inflate_;

#endif
