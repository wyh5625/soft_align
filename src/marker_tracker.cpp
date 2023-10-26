#include <ros/ros.h>
#include "moveit_commander.h"
#include "Manipulator.h"
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/chain.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float64MultiArray.h>
#include <chrono>
#include <iostream>

#include "FabricVision.h"
#include "FabricState.h"
#include "FabricTransformer.h"
#include "AlignmentClass.h"
#include "soft_align/UpdateUI.h"
#include "MarkerTracker.h"

using namespace std;
using namespace cv;




cv_bridge::CvImagePtr imageRGB, imageDepth;
Mat imageShowRGB, imageShowDepth;
bool received = false;
bool use_tracker = true;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    syncPolicy;

void chatterCallbackCVC(const sensor_msgs::ImageConstPtr &imgRGB, const sensor_msgs::ImageConstPtr &imgDepth)
{
    imageRGB = cv_bridge::toCvCopy(imgRGB, sensor_msgs::image_encodings::BGR8);
    imageDepth = cv_bridge::toCvCopy(imgDepth, sensor_msgs::image_encodings::TYPE_16UC1);
    received = true;
}




Scalar color_range[9][2]{
    {Scalar(0, 140, 140), Scalar(180, 255, 255)},   // Red
    // {Scalar(0, 90, 120), Scalar(180, 255, 255)},  // old Orange
    {Scalar(0, 100, 77), Scalar(33, 255, 255)},  // Orange  NB_lighting {Scalar(0, 90, 80), Scalar(180, 255, 255)}, 
    {Scalar(24, 50, 120), Scalar(40, 150, 255)}, // yellow
    {Scalar(60, 45, 110), Scalar(80, 255, 255)}, // green
    {Scalar(76, 160, 73), Scalar(180, 255, 180)}, // blue
    {Scalar(150, 50, 240), Scalar(180, 255, 255)}, // pink
    {Scalar(110, 60, 140), Scalar(140, 255, 255)}, // purple
    {Scalar(0, 0, 0), Scalar(180, 255, 100)},  // black
    {Scalar(0, 0, 64), Scalar(180, 255, 255)} // white
};

vector<Point> getMarkerCenterByColor(Mat src, int num){
    // num keypoints, each with a different color
    int colors[] = {0, 3, 2, 4};

    vector<Point> pts;

    for(int i = 0; i < num; i++){
        Mat frame_HSV, mask_m;
        cvtColor(src, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, color_range[colors[i]][0], color_range[colors[i]][1], mask_m);

        // offset mask_f to set boarder(1/10 of width and 1/10 of height) to 0, to avoid detecting the boarder as contour
        int offset_x = mask_m.cols / 10;
        int offset_y = mask_m.rows / 10;
        for (int i = 0; i < mask_m.rows; i++)
        {
            for (int j = 0; j < mask_m.cols; j++)
            {
                if (i < offset_y || i > mask_m.rows - offset_y || j < offset_x || j > mask_m.cols - offset_x)
                {
                    mask_m.at<uchar>(i, j) = 0;
                }
            }
        }
        
        Mat blur, canny_edge;

        medianBlur(mask_m, blur, 3);

        Canny(blur, canny_edge, 20, 140);
        Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
        dilate(canny_edge, canny_edge, element);

        SimpleBlobDetector::Params params;
        params.blobColor = 255;
        params.filterByArea = true;
        params.minArea = 10;    // Adjust this as per your requirement
        params.maxArea = 500;   // Adjust this as per your requirement

        params.filterByCircularity = true;
        params.minCircularity = 0.1;  // Lower this to be more inclusive


        
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // Detect blobs.
        vector<KeyPoint> keypoints;
        detector->detect(canny_edge, keypoints);

        if (keypoints.size() == 0){
            pts.push_back(Point(0, 0));
        }else{
            pts.push_back(Point(keypoints[0].pt.x, keypoints[0].pt.y));
        }
        
    }

    return pts;
}

Mat createWorkAreaMask(Mat src){
    // create a zero image with same size as src, rectangle from (0,100) to (src.cols - 200, src.rows-100) is 1
    Mat mask = Mat::zeros(src.size(), CV_8UC1);
    rectangle(mask, Point(100, 0), Point(src.cols - 200, src.rows-100), Scalar(255,255,255), -1);
    return mask;
}

vector<Point> getMarkerCenter(Mat src)
{
    Mat frame_HSV, mask_m;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, color_range[1][0], color_range[1][1], mask_m);


    Mat blur, canny_edge;
    medianBlur(mask_m, blur, 3);

    Canny(blur, canny_edge, 20, 140);
    Mat element = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(canny_edge, canny_edge, element);

    Mat mask_wa = createWorkAreaMask(src);
    bitwise_and(canny_edge, mask_wa, canny_edge);
    

    SimpleBlobDetector::Params params;
    params.blobColor = 255;
    params.filterByArea = true;
    params.minArea = 10;    // Adjust this as per your requirement
    params.maxArea = 500;   // Adjust this as per your requirement

    params.filterByCircularity = true;
    params.minCircularity = 0.1;  // Lower this to be more inclusive


    
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    // Detect blobs.
    vector<KeyPoint> keypoints;
    detector->detect(canny_edge, keypoints);

    printf("keypoints size: %d\n", keypoints.size());

    // show mask_m
    imshow("keypoints", canny_edge);
    // save keypoints image to subfold Debug
    std::string data_path = ros::package::getPath("soft_align") + "/data/";
    imwrite(data_path + "keypoints.png", canny_edge);
    // waitKey(0);

    vector<Point> pts;
    for (int i = 0; i < keypoints.size(); i++)
    {
        pts.push_back(Point(keypoints[i].pt.x, keypoints[i].pt.y));
    }

    return pts;
}

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

    Mat frame_HSV, mask_f, mask_m;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, color_range[8][0], color_range[8][1], mask_f);
    //inRange(frame_HSV, color_range[1][0], color_range[1][1], mask_m);

    // In mask_f pixel that is outside of rectangle of Point(100, 0), Point(src.cols - 200, src.rows - 50), set them to 0
    // for (int i = 0; i < mask_f.rows; i++)
    // {
    //     for (int j = 0; j < mask_f.cols; j++)
    //     {
    //         if (i < 0 || i > mask_f.rows - 100 || j < 100 || j > mask_f.cols - 200)
    //         {
    //             mask_f.at<uchar>(i, j) = 0;
    //         }
    //     }
    // }

    Mat mask_wa = createWorkAreaMask(src);
    bitwise_and(mask_f, mask_wa, mask_f);

    // show mask_f
    // imshow("mask_f", mask_f);
    // waitKey(0);

    //bitwise_or(mask_f, mask_m, mask_f);

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(mask_f, mask_f, element);

    Mat blur;
    medianBlur(mask_f, blur, 3);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(blur, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int id_max = getMaxAreaContourId(contours);

    // draw max area contour
    // create a white image with all channel 255
    Mat drawing = Mat::ones(src.size(), CV_8UC3) * 255;


    
    // draw zeros on by giving the left-top and right-bottom points
    rectangle(drawing, Point(100, 0), Point(src.cols - 200, src.rows-100), Scalar(0, 0, 0), -1);
    drawContours(drawing, contours, id_max, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
    
    std::string data_path = ros::package::getPath("soft_align") + "/data/";
    imwrite(data_path + "contour.png", drawing);

    // show drawing
    imshow("drawing", drawing);

    return contours[id_max];
}



int main(int argc, char **argv)
{

    // int num_marker = atoi(argv[2]);

    int PL_times = 0;
    vector<double> avg_errors; // len(avg_errors) == PL_times+1
    vector<double> iOUs;

    if (argc < 2)
    {
        std::cout << "Not enough arguments passed in!" << std::endl;
        exit(-1);
    }

    int mode = 1; // 0: manual plan; 1: auto R plan; 2: auto TR plan

    string pattern = argv[1];

    ros::init(argc, argv, "soft_tracker_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::Rate rate(30);

    message_filters::Subscriber<sensor_msgs::Image> imageRGB_sub(node, "/camera0/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> imageDepth_sub(node, "/camera0/aligned_depth_to_color/image_raw", 1);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imageRGB_sub, imageDepth_sub);
    sync.registerCallback(boost::bind(&chatterCallbackCVC, _1, _2));
    ros::ServiceClient uiClient = node.serviceClient<soft_align::UpdateUI>("/soft_align/update_ui");

    // create a publisher to publish an array of marker positions
    ros::Publisher marker_pub = node.advertise<std_msgs::Float64MultiArray>("/soft_align/mapped_markers", 1);

    // namedWindow("markers_tracker");


    /* Init vision */
    // 1. camera info
    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info_msg(new sensor_msgs::CameraInfo);
    camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera0/color/camera_info", ros::Duration(5));
    init_camera_info_rgb(camera_info_msg);
    cout << "Init camera info: OK!" << endl;

    // 2. Vision
    tf::TransformListener *listener;
    listener = new tf::TransformListener();
    try
    {
        listener->waitForTransform("/camera0_link", "/base_link", ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    Vision mVision(camera_intrin_rgb, listener, "0");

    // 3. FabricTransformer
    FabricTransformer ft;
    ft.mVision = mVision;
    string tp_file = pattern + ".txt";
    ft.initTP(tp_file);
    
    bool init = false;

    int num_cnr = ft.tp_cnrs.size();
    int thres_uncertain = 20;

    MarkerTracker tracker(num_cnr, thres_uncertain, &ft);

    vector<Point> corners_x;

    // Initialize all corners to point -1
    for(int i = 0; i < num_cnr; i++){
        corners_x.push_back(Point(-1, -1));
    }

    // std::cout << "---------------------0 !--------------" << std::endl;


    FabricState fs;
    

    while (ros::ok())
    {
        if (received)
        {
            imageRGB->image.copyTo(imageShowRGB);
            imageDepth->image.copyTo(imageShowDepth);

            vector<Point> fabric_outline = getFabricContour(imageShowRGB);
            vector<Point> markers;
            if (use_tracker){
                markers = getMarkerCenter(imageShowRGB);
            }else{
                markers = getMarkerCenterByColor(imageShowRGB, num_cnr);
            }
            std::cout << "Marker_size: " << markers.size() << std::endl;
            // std::cout << "---------------------1 !--------------" << std::endl;
            if (use_tracker){
                if (!init){
                    // fs = ft.getFabricStateFromData(markers, fabric_outline, imageShowDepth, 0.18);
                    // // fs.cnr_idx_on_ctr = ft.cnr_idx_on_ctr;
                    // corners_x = fs.cnr_x;
                    // std::cout << "---------------------2 !--------------" << std::endl;
                    tracker.init(markers, fabric_outline, imageShowDepth);
                    // std::cout << "---------------------3 !--------------" << std::endl;
                    init = true;
                }
                else{
                    // std::cout << "---------------------4 !--------------" << std::endl;
                    tracker.update(markers, fabric_outline, imageShowDepth);
                    // std::cout << "---------------------5 !--------------" << std::endl;
                    // ft.updateCornerByMarker(markers, fabric_outline, fs.cnr_x, imageShowDepth);
                }
                corners_x = tracker.corners;
                // std::cout << "---------------------6 !--------------" << std::endl;
            }else{
                fs = ft.getFabricStateFromData(markers, fabric_outline, imageShowDepth, 0.18, true);
                // fs.cnr_idx_on_ctr = ft.cnr_idx_on_ctr;
                corners_x = fs.cnr_x;
            }
            

            // publish corner pose
            std_msgs::Float64MultiArray marker_pose;
            marker_pose.data.resize(corners_x.size() * 2);
            for (int i = 0; i < corners_x.size(); i++)
            {
                marker_pose.data[i * 2] = corners_x[i].x;
                marker_pose.data[i * 2 + 1] = corners_x[i].y;
            }

            marker_pub.publish(marker_pose);

            // draw corners on image
            for (int i = 0; i < corners_x.size(); i++)
            {
                circle(imageShowRGB, corners_x[i], 3, Scalar(0, 0, 255), -1);
                // label corner id
                putText(imageShowRGB, to_string(i), corners_x[i], FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 1);
            }

            // show image
            imshow("markers_tracker", imageShowRGB);
            waitKey(1);

        }

        ros::Rate rate(30);
        ros::spinOnce();
        
    }


}

