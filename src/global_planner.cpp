#include "global_planner.h"


namespace Global_Planning{
    // 初始化函数
    void Global_Planner::init(ros::NodeHandle& nh){
        nh.param("global_planner/time_per_path", time_per_path, 1.0);
        // 重规划周期，环境变化越快，该数值越低
        nh.param("global_planner/replan_time", replan_time, 2.0);
        // 选择地图更新方式：　true代表全局点云，false代表激光雷达scan数据
        nh.param("global_planner/map_input", map_input, true);

        // 订阅 目标点
        goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 1, &Global_Planner::goal_cb, this);
        // 订阅 无人机状态
        drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, &Global_Planner::drone_state_cb, this);
        // 根据map_input选择地图更新方式
        if(map_input){
            Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/prometheus/global_planning/global_pcl", 1, &Global_Planner::Gpointcloud_cb, this);
        }else{
            laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/prometheus/global_planning/laser_scan", 1, &Global_Planner::laser_cb, this);
        }

        // 发布路径指令
        command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
        // 发布提示消息
        message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/global_planner", 10);
        // 发布路径用于显示
        path_cmd_pub = nh.advertise<nav_msgs::Path>("/prometheus/global_planning/path_cmd", 10);

        // 定时器规划器算法执行
        mainloop_timer = nh.createTimer(ros::Duration(replan_time), &Global_Planner::mainloop_cb, this);
        // 路径追踪循环，快速移动场景应当适当提高执行频率
        track_path_timer = nh.createTimer(ros::Duration(time_per_path), &Global_Planner::track_path_cb, this);


        // Astar algorithm
        Astar_ptr.reset(new Astar);
        Astar_ptr->init(nh);

        message = "A_star init.";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);


        // 规划器状态参数初始化
        exec_state = EXEC_STATE::IDLE;

        goal_ready = false;
        is_new_path = false;

        // 初始化发布的指令
        ControlCommand.header.stamp = ros::Time::now();
        ControlCommand.Mode  = prometheus_msgs::ControlCommand::Idle;
        ControlCommand.Command_ID = 0;
        ControlCommand.source = NODE_NAME;
        ControlCommand.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
        ControlCommand.Reference_State.yaw_ref = 0.0;
    }


    // 获得新目标点
    void Global_Planner::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg){
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        // set final velocity to zero
        goal_vel.setZero();

        goal_ready = true;

        message = "Goal set! (" + goal_pos(0) + ", " + goal_pos(1) + ", " + goal_pos(2) + ")";
        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
    }


    void Global_Planner::drone_state_cb(const prometheus_msgs::DroneStateConstPtr& msg){
        start_pos << msg->position[0], msg->position[1], msg->position[2];
        start_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];

        // Drone_odem is needed only when map_input is false, speedup!
        if(!map_input){
            Drone_odom.header = msg->header;
            Drone_odom.child_frame_id = "base_link";
            Drone_odom.pose.pose.position.x = msg->position[0];
            Drone_odom.pose.pose.position.y = msg->position[1];
            Drone_odom.pose.pose.position.z = msg->position[2];
            Drone_odom.pose.pose.orientation = msg->attitude_q;
            Drone_odom.twist.twist.linear.x = msg->velocity[0];
            Drone_odom.twist.twist.linear.y = msg->velocity[1];
            Drone_odom.twist.twist.linear.z = msg->velocity[2];
        }

        _DroneState = *msg;
    }


    // 根据全局点云更新地图：已知全局点云的场景、由SLAM实时获取的全局点云
    void Global_Planner::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
        /* need odom_ for center radius sensing */

        static int update_num=0;
        update_num++;

        // 此处改为根据循环时间计算的数值
        if(update_num == 10){
            // 对Astar中的地图进行更新
            Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
            // 并对地图进行膨胀
            Astar_ptr->Occupy_map_ptr->inflate_point_cloud();
            update_num = 0;
        }
    }


    // 根据2维雷达数据更新地图：2维激光雷达
    void Global_Planner::laser_cb(const sensor_msgs::LaserScanConstPtr &msg){
        /* need odom_ for center radius sensing */

        // 对Astar中的地图进行更新（laser+odom）
        Astar_ptr->Occupy_map_ptr->map_update_laser(msg, Drone_odom);
        // 并对地图进行膨胀
        Astar_ptr->Occupy_map_ptr->inflate_point_cloud();
    }


    void Global_Planner::track_path_cb(const ros::TimerEvent& e){
        if(!path_ok){
            return;
        }

        is_new_path = false;

        // 抵达终点
        if(cur_id == Num_total_wp - 1){
            ControlCommand.header.stamp = ros::Time::now();
            ControlCommand.Mode                                = prometheus_msgs::ControlCommand::Move;
            ControlCommand.Command_ID                          = ControlCommand.Command_ID + 1;
            ControlCommand.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
            ControlCommand.Reference_State.position_ref[0]     = goal_pos[0];
            ControlCommand.Reference_State.position_ref[1]     = goal_pos[1];
            ControlCommand.Reference_State.position_ref[2]     = goal_pos[2];

            command_pub.publish(ControlCommand);

            message = "Reach the goal!";
            pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);

            // 停止执行
            path_ok = false;
            // 转换状态为等待目标
            exec_state = EXEC_STATE::IDLE;
            return;
        }

        int i = cur_id;

        cout << "Moving to Waypoint: [ " << cur_id << " / " << Num_total_wp << " ] "<<endl;
        cout << "Moving to Waypoint:"   << path_cmd.poses[i].pose.position.x  << " [m] "
                                        << path_cmd.poses[i].pose.position.y  << " [m] "
                                        << path_cmd.poses[i].pose.position.z  << " [m] "<<endl;

        // 控制方式如果是走航点，则需要对无人机进行限速，保证无人机的平滑移动
        // 采用轨迹控制的方式进行追踪，期望速度 = （期望位置 - 当前位置）/预计时间；

        ControlCommand.header.stamp = ros::Time::now();
        ControlCommand.Mode                                = prometheus_msgs::ControlCommand::Move;
        ControlCommand.Command_ID                          = ControlCommand.Command_ID + 1;
        ControlCommand.Reference_State.Move_mode           = prometheus_msgs::PositionReference::TRAJECTORY;
        ControlCommand.Reference_State.position_ref[0]     = path_cmd.poses[i].pose.position.x;
        ControlCommand.Reference_State.position_ref[1]     = path_cmd.poses[i].pose.position.y;
        ControlCommand.Reference_State.position_ref[2]     = path_cmd.poses[i].pose.position.z;
        // TODO 速度这样设置是否合理？
        ControlCommand.Reference_State.velocity_ref[0]     = (path_cmd.poses[i].pose.position.x - _DroneState.position[0])/time_per_path;
        ControlCommand.Reference_State.velocity_ref[1]     = (path_cmd.poses[i].pose.position.y - _DroneState.position[1])/time_per_path;
        ControlCommand.Reference_State.velocity_ref[2]     = (path_cmd.poses[i].pose.position.z - _DroneState.position[2])/time_per_path;

        command_pub.publish(ControlCommand);

        cur_id = cur_id + 1;
    }


    // 主循环
    void Global_Planner::mainloop_cb(const ros::TimerEvent& e){
        // TODO exec_num有什么用？
        static int exec_num=0;
        exec_num++;

        switch (exec_state){
            case IDLE:{
                path_ok = false;
                if(!goal_ready){
                    if(exec_num == 10){
                        message = "Waiting for a new goal.";
                        pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
                        exec_num=0;
                    }
                }else{
                    // 获取到目标点后，生成新轨迹
                    exec_state = EXEC_STATE::PLANNING;
                    goal_ready = false;
                }
                break;
            }

            case PLANNING:{
                // 重置规划器
                Astar_ptr->reset();
                // 使用规划器执行搜索，返回搜索结果

                int astar_state;

                // Astar algorithm
                astar_state = Astar_ptr->search(start_pos, goal_pos);

                // 未寻找到路径
                if(astar_state==Astar::NO_PATH){
                    path_ok = false;
                    exec_state = EXEC_STATE::IDLE;

                    message = "Astar can't find path: goal point is occupied OR reach the max_search_num.";
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                }
                else{
                    path_ok = true;
                    is_new_path = true;
                    path_cmd = Astar_ptr->get_ros_path();
                    Num_total_wp = path_cmd.poses.size();
                    start_point_index = get_start_point_id();
                    cur_id = start_point_index;
                    exec_state = EXEC_STATE::TRACKING;

                    path_cmd_pub.publish(path_cmd);
                    message = "Get a new path!";
                    pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, message);
                }

                break;
            }
            case TRACKING:{
                // 本循环是1Hz,此处不是很精准
                if(exec_num >= replan_time){
                    exec_state = EXEC_STATE::PLANNING;
                    exec_num = 0;
                }
                break;
            }
        }
    }


    // TODO 能不能改成指针？
    int Global_Planner::get_start_point_id(void){
        // 选择与当前无人机所在位置最近的点,并从该点开始追踪
        int id = 0;
        float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - _DroneState.position[0])
                                    + abs(path_cmd.poses[0].pose.position.y - _DroneState.position[1])
                                    + abs(path_cmd.poses[0].pose.position.z - _DroneState.position[2]);

        float distance_to_wp;

        for (int j=1; j<Num_total_wp;j++){
            distance_to_wp = abs(path_cmd.poses[j].pose.position.x - _DroneState.position[0])
                                    + abs(path_cmd.poses[j].pose.position.y - _DroneState.position[1])
                                    + abs(path_cmd.poses[j].pose.position.z - _DroneState.position[2]);

            if(distance_to_wp < distance_to_wp_min){
                distance_to_wp_min = distance_to_wp;
                id = j;
            }
        }

        //　为防止出现回头的情况，此处对航点进行前馈处理
        if(id + 2 < Num_total_wp){
            id = id + 2;
        }

        return id;
    }
}
