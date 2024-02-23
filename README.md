# 无人机动态避障 ![logo200x60](https://github.com/HuaYuXiao/UAV-Dynamic-Obstacle-Avoidance-Based-on-APF/assets/117464811/88415d13-8c7c-4d5c-a3e7-04f02d7b746d)

<!--
![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FUAV-Obstacle-Avoidance-System-for-Complex-Environment-Based-on-A-and-Other-Algorithms.json%3Fcolor%3Dpink)
-->
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-2.7.17-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Matlab-2023b-salmon)
<!--
![Static Badge](https://img.shields.io/badge/CMake-3.10.2-064F8C?logo=cmake)
-->


## 简介

从[Prometheus450无人机](https://wiki.amovlab.com/public/prometheuswiki/P450%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C.html)✈搭载的[T265双目相机](https://www.intel.cn/content/www/cn/zh/products/sku/192742/intel-realsense-tracking-camera-t265/specifications.html)和[LDS-50C-3激光雷达](https://www.pacecat.com/lds-50c.html)感知环境中的障碍物，并设计[人工势场算法](https://zh.wikipedia.org/zh-cn/%E4%BA%BA%E5%B7%A5%E5%8A%BF%E5%9C%BA%E6%B3%95)实时规划运动路径，控制无人机安全稳定地到达目的地。




## 重要指令

### 室内指点飞行

#### 启动基本vio脚本

```bash
roslaunch p450_experiment p450_vio_onboard.launch
```

#### 启动控制脚本

```bash
roslaunch p450_experiment p450_vio_control.launch
```

### 室内自主避障

#### 

```bash
roslaunch p450_experiment astar_onboard.launch
```

####

```bash
roslaunch p450_experiment astar_ground.launch
```

### 室内自主降落

#### 启动自主降落脚本

```bash
roslaunch p450_experiment p450_indoor_landing_static_target.launch
```

#### 将图像检测显示出来

```bash
rqt_image_view
```


## 谢辞

- 感谢[陈亮名老师](https://faculty.sustech.edu.cn/?tagid=chenlm6&iscss=1&snapid=1&orderby=date&go=1)提供的技术指导😊！
- 感谢[哈工深MASLAB](https://github.com/HITSZ-MAS)提供的场地支持😊！


