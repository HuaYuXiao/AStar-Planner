# ![logo200x60](https://github.com/HuaYuXiao/UAV-Dynamic-Obstacle-Avoidance/assets/117464811/88415d13-8c7c-4d5c-a3e7-04f02d7b746d) 无人机动态避障

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FUAV-Dynamic-Obstacle-Avoidance.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/C%2B%2B-11-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/MATLAB-2023b-salmon)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)
<!--
![Static Badge](https://img.shields.io/badge/Python-2.7.17-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/CMake-3.10.2-064F8C?logo=cmake)
-->


从[Prometheus450无人机](https://wiki.amovlab.com/public/prometheuswiki/P450%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C.html)✈搭载的[T265双目相机](https://www.intel.cn/content/www/cn/zh/products/sku/192742/intel-realsense-tracking-camera-t265/specifications.html)和[LDS-50C-3激光雷达](https://www.pacecat.com/lds-50c.html)感知环境中的障碍物，并设计各种路径规划算法实时规划运动路径，控制无人机安全稳定地到达目的地。

**NOTICE**：
- **建图**请前往此仓库👉https://github.com/HuaYuXiao/uav_octomapping
- **定位**请前往此仓库👉https://github.com/HuaYuXiao/uav_localization



## 实物实验

### 1. 下载源码

```bash
cd ~/catkin_ws
```

```bash
git clone https://github.com/HuaYuXiao/uav_navigation.git
```


### 2. 编译安装

```bash
catkin_make -j2 -l2 install --pkg=uav_navigation
```

NOTICE：由于板载计算机性能表现不佳，此处仅使用2个线程进行编译和链接。

编译安装完成后，在.bashrc末尾加上：`source ~/catkin_ws/devel/setup.bash`（已经添加过的跳过），

```bash
gedit ~/.bashrc
```


### 3. 开始导航

```bash
roslaunch uav_navigation uav_navigation.launch
```

**2024-03-15更新**：以下指令已弃用：（可以用，但是不推荐）

```bash
roslaunch p450_experiment astar_onboard.launch
```

```bash
roslaunch p450_experiment astar_ground.launch
```

Reference:
- ⭐ [进阶功能-室内自主避障](https://wiki.amovlab.com/public/prometheuswiki/P450%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C/%E8%BF%9B%E9%98%B6%E5%8A%9F%E8%83%BD-%E5%AE%A4%E5%86%85%E8%87%AA%E4%B8%BB%E9%81%BF%E9%9A%9C.html)
- [技术分享 | Prometheus避障—A_star算法代码阅读](https://mp.weixin.qq.com/s/TR9KgxV2lFZX_4VJ_I6kAQ)
- [技术分享 | Prometheus（P450）-室内外避障](https://mp.weixin.qq.com/s/j4-Z_OIIW9ReXpfisAh37Q)


## 4. 确定起点



Reference:
- ⭐ [How to Create an Initial Pose and Goal Publisher in ROS](https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/)




## 谢辞
- 感谢**陈亮名**副教授提供的指导😊！
- 感谢**哈工深MASLAB**提供的场地与设备支持😊！
- 感谢**马健斌**、**岳江源**、**李奥淇**等师兄们的技术支持😊！
