#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER
#define NODE_NAME "Global_Planner [main]"

#include "A_star.h"

using namespace std;

namespace Global_Planning{
extern ros::Publisher message_pub;
class Global_Planner{
private:
    ros::NodeHandle global_planner_nh;

    // 参数
    int algorithm_mode;
    bool is_2D;
    double fly_height_2D;
    double safe_distance;
    double time_per_path;
    int map_input;
    double replan_time;
    bool consider_neighbour;
    bool sim_mode;
    bool map_groundtruth;

    // 本机位置
    // 邻机位置
    // 根据不同的输入（激光雷达输入、相机输入等）生成occupymap
    // 调用路径规划算法 生成路径
    // 调用轨迹优化算法 规划轨迹

    // subcribe目标点
    ros::Subscriber initialpose_sub;
    ros::Subscriber goal_sub;
    // 订阅无人机状态
    ros::Subscriber drone_state_sub;
    // 支持2维激光雷达、3维激光雷达、D435i等实体传感器
    // 支持直接输入全局已知点云
    //    传感器数据（生成地图）
    ros::Subscriber Gpointcloud_sub;
    ros::Subscriber Lpointcloud_sub;
    ros::Subscriber laserscan_sub;

    // 发布控制指令
    ros::Publisher command_pub;
    ros::Publisher path_cmd_pub;

    ros::Timer mainloop_timer;
    ros::Timer track_path_timer;
    ros::Timer safety_timer;

    // A星规划器
    Astar::Ptr Astar_ptr;

    geometry_msgs::PoseStamped initialpose;
    // 创建一个TransformStamped消息并填充数据
    geometry_msgs::TransformStamped static_transformStamped;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    prometheus_msgs::DroneState _DroneState;
    nav_msgs::Odometry Drone_odom;

    nav_msgs::Path path_cmd;
    prometheus_msgs::ControlCommand Command_Now;

    double distance_walked;
    double distance_to_goal;

    // 规划器状态
    bool odom_ready;
    bool drone_ready;
    bool sensor_ready;
    bool goal_ready; 
    bool is_safety;
    bool is_new_path;
    bool path_ok;
    int start_point_index;
    int Num_total_wp;
    int cur_id;

    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d goal_pos;
    Eigen::Vector3d goal_vel;

    float desired_yaw;
    ros::Time tra_start_time;
    float tra_running_time;
    
    // 打印的提示消息
    string message;

    // 五种状态机
    enum EXEC_STATE{
        WAIT_GOAL,
        PLANNING,
        TRACKING,
        LANDING,
    };
    EXEC_STATE exec_state;

    // 回调函数
    void initialpose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg);

    void Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laser_cb(const sensor_msgs::LaserScanConstPtr& msg);

    void safety_cb(const ros::TimerEvent& e);
    void mainloop_cb(const ros::TimerEvent& e);
    void track_path_cb(const ros::TimerEvent& e);

    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const ros::Time& begin_time);

    int get_start_point_id(void);
    
public:
    Global_Planner(void):
        global_planner_nh("~")
    {}~Global_Planner(){}

    void init(ros::NodeHandle& nh);
};
}
#endif
