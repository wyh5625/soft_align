#include <ros/ros.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/chain.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "moveit_commander.h"
#include "Manipulator.h"
#include "soft_align/UpdateUI.h"

using namespace std;
using namespace cv;


cv_bridge::CvImagePtr imageRGB, imageDepth;
Mat imageShowRGB, imageShowDepth;
bool received = false;

vector<Point> fs_start, fs_end, fs_c, fs_t;
vector<Point> pl_t, pl_r, arrowedline_t, arrowedcurve_r;
vector<Point> sm_lines;
vector<Point> cnr_t;
vector<int> matched;
string text = "";


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    syncPolicy;

void chatterCallbackCVC(const sensor_msgs::ImageConstPtr &imgRGB, const sensor_msgs::ImageConstPtr &imgDepth)
{
    imageRGB = cv_bridge::toCvCopy(imgRGB, sensor_msgs::image_encodings::BGR8);
    imageDepth = cv_bridge::toCvCopy(imgDepth, sensor_msgs::image_encodings::TYPE_16UC1);
    received = true;
}

void msg2point(vector<geometry_msgs::Point> msg, vector<Point> &data){
    data.clear();
    for(int i = 0; i < msg.size(); i ++){
        data.push_back(Point(int(msg[i].x), int(msg[i].y)));
    }
}

void msg2array(vector<int8_t> msg, vector<int> &data){
    data.clear();
    for(int i = 0; i < msg.size(); i ++){
        data.push_back(msg[i]);
    }
}

bool updateUI(soft_align::UpdateUI::Request &req,
                      soft_align::UpdateUI::Response &res)
{
    msg2point(req.fs_start, fs_start);
    msg2point(req.fs_end, fs_end);
    msg2point(req.fs_t, fs_t);
    msg2point(req.fs_c, fs_c);
    msg2point(req.pl_t, pl_t);
    msg2point(req.pl_r, pl_r);
    msg2point(req.arrowedline_t, arrowedline_t);
    msg2point(req.arrowedcurve_r, arrowedcurve_r);
    msg2point(req.sm_lines, sm_lines);
    msg2point(req.cnr_t, cnr_t);
    msg2array(req.matched_t, matched);
    text = req.description;

    res.update_success = true;
    return true;
}

void showCtr(Mat& frame, vector<Point> ctr, Scalar color = Scalar::all(255)){
    if(!ctr.empty()){
        vector<vector<Point>> ctr_set;
        cv::approxPolyDP(ctr, ctr, 3, true);
        ctr_set.push_back(ctr);
        // draw dot for each contour point
        for (int i = 0; i < ctr.size(); i++)
        {
            cv::circle(frame, ctr[i], 1, color, -1);
        }

        // for each consecutive contour point, draw circle between them distributed evenly with a step
        int step = 10;
        for (int i = 0; i < ctr.size() - 1; i++)
        {
            // number of circles to draw between two points depends on the distance between them
            int n = int(cv::norm(ctr[i] - ctr[i + 1]) / step);

            for (int j = 0; j < n; j++)
            {
                double t = j / (n*1.0);
                Point p = (1 - t) * ctr[i] + t * ctr[i + 1];
                cv::circle(frame, p, 1, color, -1);
            }
        }

        // Consider circles between first and last contour points
        int n = int(cv::norm(ctr[0] - ctr[ctr.size() - 1]) / step);
        for (int j = 0; j < n; j++)
        {
            double t = j / (n*1.0);
            Point p = (1 - t) * ctr[0] + t * ctr[ctr.size() - 1];
            cv::circle(frame, p, 1, color, -1);
        }

        // draw solid contour

        // cv::drawContours(frame, ctr_set, 0, color, 1);
    }
}

void showTarget(Mat& frame, vector<Point> p_target, vector<int> matched, Scalar matched_color = Scalar(0,255,0), Scalar unmatched_color = Scalar(0,0,255)){
    for(int i = 0; i < p_target.size(); i ++){
        if(matched[i] == 1){
            circle(frame, p_target[i], 6, matched_color, -1);
        }
        else{
            circle(frame, p_target[i], 6, unmatched_color, -1);
        }
    }
}

void showArrow(Mat& frame, vector<Point> line, Scalar color = Scalar::all(255)){
    if(!line.empty())
        arrowedLine(frame, line[0], line[1], color);
}

void showArrowedCurve(Mat& frame, vector<Point> pointList, Scalar color = Scalar::all(255)){
    if(!pointList.empty()){
        polylines(frame, pointList, false, color);
        arrowedLine(frame, pointList[pointList.size() - 2], pointList[pointList.size() - 1], color);
    }
    
}

void showSmoothLines(Mat& frame, vector<Point> sm_lines, Scalar c1 = Scalar::all(255), Scalar c2 = Scalar::all(255)){
    if(!sm_lines.empty()){
        line(frame, sm_lines[0], sm_lines[1], c1);
        line(frame, sm_lines[2], sm_lines[3], c2);
        arrowedLine(frame, (sm_lines[0] + sm_lines[1])/2, (sm_lines[2] + sm_lines[3])/2, Scalar::all(255));
    }
}

void showUI(Mat& frame){
    // showCtr(frame, fs_start, Scalar(0,255,0));
    showCtr(frame, fs_end, Scalar(0,255,255));
    // showCtr(frame, fs_t, Scalar(255,255,255));
    // showCtr(frame, fs_c, Scalar(0,255,0));
    // showCtr(frame, pl_t, Scalar(255,255,255));
    // showCtr(frame, pl_r, Scalar(255,0,0));
    showTarget(frame, cnr_t, matched);

    // showArrow(frame, arrowedline_t, Scalar(0,0,255));
    // showArrowedCurve(frame, arrowedcurve_r, Scalar(0,0,255));
    // showSmoothLines(frame, sm_lines, Scalar(0,0,255), Scalar(0,255,0));
    // show text
    // putText(frame, text, Point(50,50), FONT_HERSHEY_COMPLEX, 1, Scalar(255,0,0), 1);
}

Scalar color_range[4][2]{
    {Scalar(0, 43, 46), Scalar(10, 255, 255)},   // Red
    {Scalar(0, 90, 120), Scalar(180, 255, 255)}, // Orange
    {Scalar(87, 50, 60), Scalar(109, 185, 255)}, // blue
    {Scalar(0, 0, 0), Scalar(180, 255, 80)}      // black

};

int getMaxAreaContourId(vector<vector<Point>> contours)
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++)
    {
        double newArea = contourArea(contours.at(j));
        if (newArea > maxArea)
        {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    }     // End for
    return maxAreaContourId;
}


vector<Point> getFabricContour(Mat src)
{

    Mat frame_HSV, mask_f;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, color_range[3][0], color_range[3][1], mask_f);

    Mat blur;
    medianBlur(mask_f, blur, 3);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(blur, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int id_max = getMaxAreaContourId(contours);

    return contours[id_max];
}

vector<Point> getMarkerCenter(Mat src)
{
    Mat frame_HSV, mask_m;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, color_range[1][0], color_range[1][1], mask_m);

    Mat blur, canny_edge;
    medianBlur(mask_m, blur, 3);

    Canny(blur, canny_edge, 20, 140);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(canny_edge, canny_edge, element);

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
    // Detect blobs.
    vector<KeyPoint> keypoints;
    detector->detect(canny_edge, keypoints);

    vector<Point> pts;
    for (int i = 0; i < keypoints.size(); i++)
    {
        pts.push_back(Point(keypoints[i].pt.x, keypoints[i].pt.y));
    }

    return pts;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "soft_align_monitor", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::Rate rate(30);

    message_filters::Subscriber<sensor_msgs::Image> imageRGB_sub(node, "/camera0/color/image_raw", 1000);
    message_filters::Subscriber<sensor_msgs::Image> imageDepth_sub(node, "/camera0/aligned_depth_to_color/image_raw", 1000);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imageRGB_sub, imageDepth_sub);
    sync.registerCallback(boost::bind(&chatterCallbackCVC, _1, _2));

    ros::ServiceServer update_server = node.advertiseService("/soft_align/update_ui", updateUI);

    while (ros::ok())
    {
        if (received)
        {
            imageRGB->image.copyTo(imageShowRGB);
            imageDepth->image.copyTo(imageShowDepth);


            // vector<Point> ctr = getFabricContour(imageShowRGB);
            // vector<Point> markers = getMarkerCenter(imageShowRGB);

            // for(int i = 0; i < markers.size(); i++){
            //     circle(imageShowRGB, markers[i], 3, Scalar::all(255), -1);
            // }

            // vector<vector<Point>> ctr_set;
            // ctr_set.push_back(ctr);

            // drawContours(imageShowRGB, ctr_set, 0, Scalar::all(255));

            // Show fs_start, fs_end, fs_t, fs_c
            // Show pushLine_t, pushLine_r, arrowedLine_t, arrowedCurve_r
            // Show smoothing line

            // Aways show: fs_start, fs_end, fs_c
            // Translation: fs_c, pushLine_t, fs_t, arrowedLine_t
            // Rotation: fs_c, fs_end, pushLine_r, arrowedCurve_r
            // Smoothing: smoothing line
            showUI(imageShowRGB);

            imshow("im_rgb", imageShowRGB);
            waitKey(3);

            received = false;
        }

        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}