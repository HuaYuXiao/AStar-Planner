# ![logo200x60](https://github.com/HuaYuXiao/UAV-Dynamic-Obstacle-Avoidance/assets/117464811/88415d13-8c7c-4d5c-a3e7-04f02d7b746d) AStar-Planner

阿木P450导航包，modified from [global_planning](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/planning/global_planning)

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fuav_navigation.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)


## Compilation

```bash
catkin_make install --source src/uav_navigation --build build/uav_navigation
```


## Launch

```bash
roslaunch uav_navigation uav_experienment.launch
```

Reference:
- ⭐ [进阶功能-室内自主避障](https://wiki.amovlab.com/public/prometheuswiki/P450%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C/%E8%BF%9B%E9%98%B6%E5%8A%9F%E8%83%BD-%E5%AE%A4%E5%86%85%E8%87%AA%E4%B8%BB%E9%81%BF%E9%9A%9C.html)
- [技术分享 | Prometheus避障—A_star算法代码阅读](https://mp.weixin.qq.com/s/TR9KgxV2lFZX_4VJ_I6kAQ)
- [技术分享 | Prometheus（P450）-室内外避障](https://mp.weixin.qq.com/s/j4-Z_OIIW9ReXpfisAh37Q)


## 确定起点

Reference:
- ⭐ [How to Create an Initial Pose and Goal Publisher in ROS](https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/)



## 致謝
- 感谢**陈亮名**副教授提供的指导😊！
- 感谢**哈工深MASLAB**提供的场地与设备支持😊！
- 感谢**马健斌**、**岳江源**、**李奥淇**等师兄们的技术支持😊！
