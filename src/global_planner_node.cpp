#include "global_planner.h"

using namespace Global_Planning;

double safe_distance;
double time_per_path;
int map_input;
double replan_time;
double lambda_heu_;
int max_search_num;
double resolution_;
double inflate_;

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner_main");

    ros::NodeHandle nh("~");

    // 安全距离，若膨胀距离设置已考虑安全距离，建议此处设为0
    nh.param("global_planner/safe_distance", safe_distance, 0.05);
    nh.param("global_planner/time_per_path", time_per_path, 1.5);
    // 选择地图更新方式：　0代表全局点云，１代表局部点云，２代表激光雷达scan数据
    nh.param("global_planner/map_input", map_input, 0);
    // 重规划频率
    nh.param("global_planner/replan_time", replan_time, 2.0);
    // 规划搜索相关参数 加速引导参数
    nh.param("astar/lambda_heu", lambda_heu_, 2.0);
    // 最大搜索节点数
    nh.param("astar/allocate_num", max_search_num, 200);
    // 地图参数 地图分辨率，单位：米
    nh.param("map/resolution", resolution_, 0.05);
    // 地图膨胀距离，单位：米
    nh.param("map/inflate", inflate_, 0.3);

    Global_Planner global_planner;
    global_planner.init(nh);

    ros::spin();

    return 0;
}
