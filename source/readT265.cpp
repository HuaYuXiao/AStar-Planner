#include<iostream>
#include<string>

#include<librealsense2/rs.hpp>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc,char** argv)
{
    rs2::config cfg;

    // 使能 左右目图像数据
    cfg.enable_stream(RS2_STREAM_FISHEYE,1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE,2, RS2_FORMAT_Y8);

    // 使能 传感器的POSE和6DOF IMU数据
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    rs2::pipeline pipe;
    pipe.start(cfg);

    rs2::frameset data;

    while (1)
   {
    data = pipe.wait_for_frames();
	// Get a frame from the pose stream
	auto f = data.first_or_default(RS2_STREAM_POSE);
	auto pose = f.as<rs2::pose_frame>().get_pose_data();
	
	cout<<"px: "<<pose.translation.x<<"   py: "<<pose.translation.y<<"   pz: "<<pose.translation.z<<
	"vx: "<<pose.velocity.x<<"   vy: "<<pose.velocity.y<<"   vz: "<<pose.velocity.z<<endl;
	cout<<"ax: "<<pose.acceleration.x<<"   ay: "<<pose.acceleration.y<<"   az: "<<pose.acceleration.z<<
	"gx: "<<pose.angular_velocity.x<<"   gy: "<<pose.angular_velocity.y<<"   gz: "<<pose.angular_velocity.z<<endl;

    rs2::frame image_left = data.get_fisheye_frame(1);
    rs2::frame image_right = data.get_fisheye_frame(2);

      if (!image_left || !image_right)
          break;

      cv::Mat cv_image_left(cv::Size(848, 800), CV_8U, (void*)image_left.get_data(), cv::Mat::AUTO_STEP);
      cv::Mat cv_image_right(cv::Size(848, 800), CV_8U, (void*)image_right.get_data(), cv::Mat::AUTO_STEP);

      cv::imshow("left", cv_image_left);
      cv::imshow("right", cv_image_right);
      cv::waitKey(1);
    }

    return 0;
}
