#ifndef _TOOLS_H
#define _TOOLS_H

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
