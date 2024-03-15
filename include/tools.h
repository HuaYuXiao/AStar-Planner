#ifndef _TOOLS_H
#define _TOOLS_H


#include <iostream>
#include <algorithm>

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

#include "prometheus_msgs/PositionReference.h"
#include "prometheus_msgs/Message.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/Message.h"

#include "message_utils.h"


namespace Global_Planning{
    extern ros::Publisher message_pub;
}
#endif