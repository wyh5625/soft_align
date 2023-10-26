#include <realsense_ros/camera_intrin.h>
#include <sensor_msgs/CameraInfo.h>




realsense_ros::camera_intrin camera_intrin_rgb;
void init_camera_info_rgb(boost::shared_ptr<sensor_msgs::CameraInfo const> msg)
{
    camera_intrin_rgb.ppx = msg->K[2];
    camera_intrin_rgb.ppy = msg->K[5];
    camera_intrin_rgb.fx = msg->K[0];
    camera_intrin_rgb.fy = msg->K[4];
    for (int i = 0; i < 5; i++)
        camera_intrin_rgb.coeffs.push_back(0);
    camera_intrin_rgb.dev_depth_scale = 0.001;
}