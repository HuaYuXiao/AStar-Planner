- [Intel Realsense T265使用教程](https://blog.csdn.net/crp997576280/article/details/109544456)



```
... logging to /home/amov/.ros/log/ba34ff88-91bb-11ee-9d82-48b02d67bcf5/roslaunch-amov-15340.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://amov:44913/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.13
 * /t265/realsense2_camera/accel_fps: 62
 * /t265/realsense2_camera/accel_frame_id: t265_accel_frame
 * /t265/realsense2_camera/accel_optical_frame_id: t265_accel_optica...
 * /t265/realsense2_camera/align_depth: False
 * /t265/realsense2_camera/aligned_depth_to_color_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/aligned_depth_to_fisheye1_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/aligned_depth_to_fisheye2_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/aligned_depth_to_fisheye_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/aligned_depth_to_infra1_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/aligned_depth_to_infra2_frame_id: t265_aligned_dept...
 * /t265/realsense2_camera/allow_no_texture_points: False
 * /t265/realsense2_camera/base_frame_id: t265_link
 * /t265/realsense2_camera/calib_odom_file: 
 * /t265/realsense2_camera/clip_distance: -1.0
 * /t265/realsense2_camera/color_fps: 30
 * /t265/realsense2_camera/color_frame_id: t265_color_frame
 * /t265/realsense2_camera/color_height: 480
 * /t265/realsense2_camera/color_optical_frame_id: t265_color_optica...
 * /t265/realsense2_camera/color_width: 640
 * /t265/realsense2_camera/depth_fps: 30
 * /t265/realsense2_camera/depth_frame_id: t265_depth_frame
 * /t265/realsense2_camera/depth_height: 480
 * /t265/realsense2_camera/depth_optical_frame_id: t265_depth_optica...
 * /t265/realsense2_camera/depth_width: 640
 * /t265/realsense2_camera/device_type: t265
 * /t265/realsense2_camera/enable_accel: True
 * /t265/realsense2_camera/enable_color: True
 * /t265/realsense2_camera/enable_depth: True
 * /t265/realsense2_camera/enable_fisheye1: False
 * /t265/realsense2_camera/enable_fisheye2: False
 * /t265/realsense2_camera/enable_fisheye: False
 * /t265/realsense2_camera/enable_gyro: True
 * /t265/realsense2_camera/enable_infra1: False
 * /t265/realsense2_camera/enable_infra2: False
 * /t265/realsense2_camera/enable_infra: False
 * /t265/realsense2_camera/enable_pointcloud: False
 * /t265/realsense2_camera/enable_pose: True
 * /t265/realsense2_camera/enable_sync: False
 * /t265/realsense2_camera/filters: 
 * /t265/realsense2_camera/fisheye1_frame_id: t265_fisheye1_frame
 * /t265/realsense2_camera/fisheye1_optical_frame_id: t265_fisheye1_opt...
 * /t265/realsense2_camera/fisheye2_frame_id: t265_fisheye2_frame
 * /t265/realsense2_camera/fisheye2_optical_frame_id: t265_fisheye2_opt...
 * /t265/realsense2_camera/fisheye_fps: 30
 * /t265/realsense2_camera/fisheye_frame_id: t265_fisheye_frame
 * /t265/realsense2_camera/fisheye_height: 800
 * /t265/realsense2_camera/fisheye_optical_frame_id: t265_fisheye_opti...
 * /t265/realsense2_camera/fisheye_width: 848
 * /t265/realsense2_camera/gyro_fps: 200
 * /t265/realsense2_camera/gyro_frame_id: t265_gyro_frame
 * /t265/realsense2_camera/gyro_optical_frame_id: t265_gyro_optical...
 * /t265/realsense2_camera/imu_optical_frame_id: t265_imu_optical_...
 * /t265/realsense2_camera/infra1_frame_id: t265_infra1_frame
 * /t265/realsense2_camera/infra1_optical_frame_id: t265_infra1_optic...
 * /t265/realsense2_camera/infra2_frame_id: t265_infra2_frame
 * /t265/realsense2_camera/infra2_optical_frame_id: t265_infra2_optic...
 * /t265/realsense2_camera/infra_fps: 30
 * /t265/realsense2_camera/infra_height: 480
 * /t265/realsense2_camera/infra_width: 640
 * /t265/realsense2_camera/initial_reset: False
 * /t265/realsense2_camera/json_file_path: 
 * /t265/realsense2_camera/linear_accel_cov: 0.01
 * /t265/realsense2_camera/odom_frame_id: t265_odom_frame
 * /t265/realsense2_camera/pointcloud_texture_index: 0
 * /t265/realsense2_camera/pointcloud_texture_stream: RS2_STREAM_COLOR
 * /t265/realsense2_camera/pose_frame_id: t265_pose_frame
 * /t265/realsense2_camera/pose_optical_frame_id: t265_pose_optical...
 * /t265/realsense2_camera/publish_odom_tf: True
 * /t265/realsense2_camera/publish_tf: True
 * /t265/realsense2_camera/rosbag_filename: 
 * /t265/realsense2_camera/serial_no: 
 * /t265/realsense2_camera/tf_publish_rate: 0.0
 * /t265/realsense2_camera/topic_odom_in: t265/odom_in
 * /t265/realsense2_camera/unite_imu_method: 
 * /t265/realsense2_camera/usb_port_id: 

NODES
  /t265/
    realsense2_camera (nodelet/nodelet)
    realsense2_camera_manager (nodelet/nodelet)

auto-starting new master
process[master]: started with pid [15356]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to ba34ff88-91bb-11ee-9d82-48b02d67bcf5
process[rosout-1]: started with pid [15369]
started core service [/rosout]
process[t265/realsense2_camera_manager-2]: started with pid [15386]
process[t265/realsense2_camera-3]: started with pid [15387]
[ INFO] [1701594600.228369605]: Initializing nodelet with 4 worker threads.
[ INFO] [1701594600.634535432]: RealSense ROS v2.2.15
[ INFO] [1701594600.634636003]: Built with LibRealSense v2.36.0
[ INFO] [1701594600.634748657]: Running with LibRealSense v2.36.0
[ INFO] [1701594600.727413287]:  
[ INFO] [1701594600.751834864]: Device with serial number 201222111361 was found.

[ INFO] [1701594600.751974705]: Device with physical ID 2-1.3-3 was found.
[ INFO] [1701594600.752048141]: Device with name Intel RealSense T265 was found.
[ INFO] [1701594600.753714777]: Device with port number 2-1.3 was found.
[ INFO] [1701594600.790266857]: No calib_odom_file. No input odometry accepted.
[ INFO] [1701594600.790458156]: getParameters...
[ INFO] [1701594601.340151260]: setupDevice...
[ INFO] [1701594601.340284643]: JSON file is not provided
[ INFO] [1701594601.340352298]: ROS Node Namespace: t265
[ INFO] [1701594601.340424744]: Device Name: Intel RealSense T265
[ INFO] [1701594601.340486306]: Device Serial No: 201222111361
[ INFO] [1701594601.340560106]: Device physical port: 2-1.3-3
[ INFO] [1701594601.340624949]: Device FW version: 0.2.0.951
[ INFO] [1701594601.340683333]: Device Product ID: 0x0B37
[ INFO] [1701594601.340732395]: Enable PointCloud: Off
[ INFO] [1701594601.340780154]: Align Depth: Off
[ INFO] [1701594601.340826299]: Sync Mode: Off
[ INFO] [1701594601.340909943]: Device Sensors: 
[ INFO] [1701594601.341223167]: Tracking Module was found.
[ INFO] [1701594601.341333633]: (Depth, 0) sensor isn't supported by current device! -- Skipping...
[ INFO] [1701594601.341386497]: (Color, 0) sensor isn't supported by current device! -- Skipping...
[ INFO] [1701594601.341471287]: num_filters: 0
[ INFO] [1701594601.341516026]: Setting Dynamic reconfig parameters.
[ INFO] [1701594601.413737755]: Done Setting Dynamic reconfig parameters.
[ INFO] [1701594601.414111290]: setupPublishers...
[ INFO] [1701594601.429316215]: setupStreams...
[ INFO] [1701594601.429529388]: insert Gyro to Tracking Module
[ INFO] [1701594601.429700270]: insert Accel to Tracking Module
[ INFO] [1701594601.429806779]: insert Pose to Tracking Module
[ INFO] [1701594601.461938941]: SELECTED BASE:Pose, 0
[ INFO] [1701594601.482328562]: RealSense Node Is Up!
```

```
header: 
  seq: 50627
  stamp: 
    secs: 1701594855
    nsecs:  56689978
  frame_id: "t265_odom_frame"
child_frame_id: "t265_pose_frame"
pose: 
  pose: 
    position: 
      x: 7.40704563214e-05
      y: -0.000107858519186
      z: -6.98743315297e-05
    orientation: 
      x: 0.0116495192051
      y: 0.047474257648
      z: 0.000143731449498
      w: 0.998804569244
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0
.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
twist: 
  twist: 
    linear: 
      x: 0.00128825833631
      y: -0.00328097023471
      z: -0.0028903519471
    angular: 
      x: -0.00560505770556
      y: 0.000531680910988
      z: -0.000608012573227
  covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0
.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
---
```
