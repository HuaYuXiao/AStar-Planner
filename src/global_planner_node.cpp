#include "global_planner.h"

using namespace Global_Planning;

bool is_2D;
double fly_height_2D;
double safe_distance;
double time_per_path;
int map_input;
double replan_time;
bool sim_mode;
bool map_groundtruth;
double lambda_heu_;
int max_search_num;
double resolution_;
double inflate_;

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner");

    ros::NodeHandle nh("~");

    // 2d参数 1代表2D平面规划及搜索,0代表3D
    nh.param("global_planner/is_2D", is_2D, true);
    // 2D规划时,定高高度
    nh.param("global_planner/fly_height_2D", fly_height_2D, 0.4);
    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("global_planner/safe_distance", safe_distance, 0.05);
    nh.param("global_planner/time_per_path", time_per_path, 1.5);
    // 选择地图更新方式：　0代表全局点云，１代表局部点云，２代表激光雷达scan数据
    nh.param("global_planner/map_input", map_input, 0);
    // 重规划频率
    nh.param("global_planner/replan_time", replan_time, 2.0);
    // 是否为仿真模式
    nh.param("global_planner/sim_mode", sim_mode, false);
    nh.param("global_planner/map_groundtruth", map_groundtruth, false);
    // 规划搜索相关参数 加速引导参数
    nh.param("astar/lambda_heu", lambda_heu_, 2.0);
    // 最大搜索节点数
    nh.param("astar/allocate_num", max_search_num, 200);
    // 地图参数 地图分辨率，单位：米
    nh.param("map/resolution", resolution_, 0.05);
    // 地图膨胀距离，单位：米
    nh.param("map/inflate", inflate_,  0.3);

    Global_Planner global_planner;
    global_planner.init(nh);

    ros::spin();

    return 0;
}
