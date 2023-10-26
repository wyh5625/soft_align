#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <realsense_ros/camera_intrin.h>


class Vision
{
public:
    realsense_ros::camera_intrin mIntrin;
    tf::TransformListener *tfListener;
    std::string cameraNo;

    Vision(realsense_ros::camera_intrin ci, tf::TransformListener *tfl, std::string camera_name)
    {
        mIntrin = ci;
        tfListener = tfl;
        cameraNo = camera_name;
    }

    Vision()
    {
    }

    cv::Point3f deproject(cv::Point pixel, float depth_origin, bool is_depth = false)
    {
        cv::Point3f point;
        float x = (pixel.x - mIntrin.ppx) / mIntrin.fx;
        float y = (pixel.y - mIntrin.ppy) / mIntrin.fy;
        if (is_depth)
        {
            float r2 = x * x + y * y;
            float f = 1 + mIntrin.coeffs[0] * r2 + mIntrin.coeffs[1] * r2 * r2 + mIntrin.coeffs[4] * r2 * r2 * r2;
            float ux = x * f + 2 * mIntrin.coeffs[2] * x * y + mIntrin.coeffs[3] * (r2 + 2 * x * x);
            float uy = y * f + 2 * mIntrin.coeffs[3] * x * y + mIntrin.coeffs[2] * (r2 + 2 * y * y);
            x = ux;
            y = uy;
        }
        double depth = depth_origin * mIntrin.dev_depth_scale;
        point.x = depth * x;
        point.y = depth * y;
        point.z = depth;
        return point;
    }

    cv::Point3f getPointInCameraFrame(cv::Point pt, cv::Mat image_depth)
    {
        // depth may be wrong due to noise, we re-estimate it by considering depths of pixels near it.
        std::vector<double> depths;
        // 9x9=81 pixels
        for (int i = -4; i < 5; i++)
        {
            for (int j = -4; j < 5; j++)
            {
                depths.push_back(image_depth.at<ushort>(cv::Point(pt.x + i, pt.y + j)));
            }
        }
        sort(depths.begin(), depths.end());
        // use depths in the middle
        double tot = 0;
        int n = 0;
        for (int i = 36; i < 45; i++)
        {
            tot += depths[i];
            n++;
        }
        double depth = tot / n;

        cv::Point3f centroid = deproject(pt, depth);
        return centroid;
    }

    cv::Point3f cameraToWorld(cv::Point3f cp)
    {
        cv::Point3f wp;
        geometry_msgs::PointStamped camera_point, global_point;
        camera_point.header.frame_id = "camera" + cameraNo + "_link";
        camera_point.header.stamp = ros::Time();

        camera_point.point.x = cp.x;
        camera_point.point.y = cp.y;
        camera_point.point.z = cp.z;
        // tf::TransformListener listener;
        tfListener->transformPoint("base_link", camera_point, global_point);

        wp.x = global_point.point.x;
        wp.y = global_point.point.y;
        wp.z = global_point.point.z;

        return wp;
    }

    cv::Point3f getWorldCoordinate(cv::Point pt, cv::Mat image_depth)
    {
        cv::Point3f pt3f = getPointInCameraFrame(pt, image_depth);
        cv::Point3f wp = cameraToWorld(pt3f);
        return wp;
    }

    cv::Point3f worldToCamera(cv::Point3f wp)
    {
        geometry_msgs::PointStamped camera_point, global_point;
        global_point.header.frame_id = "base_link";
        global_point.header.stamp = ros::Time();

        global_point.point.x = wp.x;
        global_point.point.y = wp.y;
        global_point.point.z = wp.z;
        // tf::TransformListener listener;
        // listener.waitForTransform("/camera" + cameraNo, "/base_link", ros::Time::now(), ros::Duration(3.0));

        tfListener->transformPoint("camera" + cameraNo + "_link", global_point, camera_point);

        cv::Point3f cp(camera_point.point.x, camera_point.point.y, camera_point.point.z);
        return cp;
    }

    cv::Point project_point_to_pixel(const cv::Point3f cp)
    {
        // assert(intrin->model != RS2_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image

        float x = cp.x / cp.z, y = cp.y / cp.z;

        float r2 = x * x + y * y;
        float f = 1 + mIntrin.coeffs[0] * r2 + mIntrin.coeffs[1] * r2 * r2 + mIntrin.coeffs[4] * r2 * r2 * r2;
        x *= f;
        y *= f;
        float dx = x + 2 * mIntrin.coeffs[2] * x * y + mIntrin.coeffs[3] * (r2 + 2 * x * x);
        float dy = y + 2 * mIntrin.coeffs[3] * x * y + mIntrin.coeffs[2] * (r2 + 2 * y * y);
        x = dx;
        y = dy;

        cv::Point p;

        p.x = floor(x * mIntrin.fx + mIntrin.ppx);
        p.y = floor(y * mIntrin.fy + mIntrin.ppy);

        return p;
    }

    cv::Point getPixel(cv::Point3f pt)
    {
        cv::Point3f cwp = worldToCamera(pt);
        cv::Point pp = project_point_to_pixel(cwp);

        return pp;
    }
};