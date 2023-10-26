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
// #include <std_msgs/Int8.h>
#include <chrono>
#include <iostream>

#include "FabricVision.h"
#include "FabricState.h"
#include "FabricTransformer.h"
#include "AlignmentClass.h"
#include "soft_align/UpdateUI.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

#define PL_LENGTH 0.18
#define PL_WIDTH 0.01
#define PL_OFFSET 0.01
#define ALLOWANCE 0.01
// #define SURFACE_HEIGHT 0.5
#define GRIPPER_OFFSET -0.16
#define STICKING_HIGHT 0
#define SURFACE_HEIGHT 0.1173

bool get_marker_from_topic = true;
bool use_primary = false;
bool waypoint_mode = true;

cv_bridge::CvImagePtr imageRGB, imageDepth;
Mat imageShowRGB, imageShowDepth;
bool received = false;

FabricTransformer ft;
FabricState fs_start, fs_end, fs_t, fs_curr;
FabricState pushLine_t, pushLine_r;
Point plt_pos(100, 100), plr_pos(100, 150);

vector<FabricState *> clickableObjects;
vector<bool> clicked;
vector<Point> cnr_marker;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    syncPolicy;

void chatterCallbackCVC(const sensor_msgs::ImageConstPtr &imgRGB, const sensor_msgs::ImageConstPtr &imgDepth)
{
    imageRGB = cv_bridge::toCvCopy(imgRGB, sensor_msgs::image_encodings::BGR8);
    imageDepth = cv_bridge::toCvCopy(imgDepth, sensor_msgs::image_encodings::TYPE_16UC1);
    received = true;
}

// Vision part //

// Planner part //
void T_R_Planner()
{
}

void DrawFabricFeatures_Contour(Mat &canvas, vector<Point> ctr_x, Scalar color = Scalar(255, 255, 255))
{
    if (!ctr_x.empty())
    {
        vector<vector<Point>> ctr_set;
        ctr_set.push_back(ctr_x);

        drawContours(canvas, ctr_set, 0, color);
    }
}

void DrawFabricFeatures_RC(Mat &canvas, Point3f rc, Scalar color = Scalar(0, 0, 255))
{
    if (rc != Point3f(0, 0, 0))
    {
        Point p = ft.mVision.getPixel(rc);
        circle(canvas, p, 5, color, -1);
    }
}

void DrawFabricFeatures_Path(Mat &canvas, FabricState fs_0, FabricState fs_1, double step, Scalar color = Scalar(255, 255, 255))
{

    vector<vector<Point>> ctr_set;
    double distance = norm(fs_1.cnr[0] - fs_0.cnr[0]);
    Point2f dir = (fs_1.cnr[0] - fs_0.cnr[0]) / distance;
    double trail_next = step;
    while (trail_next < distance)
    {
        FabricState fs_next;
        fs_0.copyTo(fs_next);
        Point2f mv = dir * trail_next;
        ft.transform(fs_next, mv);

        ctr_set.push_back(fs_next.ctr_x);
        trail_next += step;
    }
    drawContours(canvas, ctr_set, -1, color);
}

void DrawTranslationArrow(Mat &canvas, FabricState pl, Point mv, Scalar color = Scalar(255, 255, 255))
{
    Point pl_center = (pl.cnr_x[0] + pl.cnr_x[1] + pl.cnr_x[2] + pl.cnr_x[3]) / 4;
    arrowedLine(canvas, pl_center, pl_center + mv, color);
}

vector<Point> dotOnCurve(Point3f center, Point2f tail, double rad_r, int density)
{
    vector<Point2f> pointList;
    vector<Point> pointList_x;

    Point2f center_2d(center.x, center.y);
    // calculate rotate angle
    Point2f v_t = tail - center_2d;

    for (int i = 0; i <= density; i++)
    {
        double rad = i * rad_r / density;

        double R[2][2] = {
            {cos(rad), -sin(rad)},
            {sin(rad), cos(rad)}};

        Point2f dot = center_2d + Point2f(R[0][0] * v_t.x + R[0][1] * v_t.y, R[1][0] * v_t.x + R[1][1] * v_t.y);
        Point dot_x = ft.mVision.getPixel(Point3f(dot.x, dot.y, center.z));
        pointList.push_back(dot);
        pointList_x.push_back(dot_x);
    }
    return pointList_x;
}

void DrawRotationArrow(Mat &canvas, Point3f center, Point2f tail, double rad_r, Scalar color = Scalar(255, 255, 255))
{
    int dot_count = 10;
    // vector<Point2f> pointList;

    vector<Point> pointList_x = dotOnCurve(center, tail, rad_r, dot_count);

    polylines(canvas, pointList_x, false, color);
    arrowedLine(canvas, pointList_x[dot_count - 1], pointList_x[dot_count], color);
}

void DrawRotation_Path()
{
}

void DrawPushLine(Mat &canvas, FabricState pl_state)
{
}

void initPushLine(FabricState &pl_state, double l, double w, Point center)
{
    Point3f center_w = ft.mVision.getWorldCoordinate(center, imageShowDepth);

    pl_state.center.x = center_w.x;
    pl_state.center.y = center_w.y;

    pl_state.height = SURFACE_HEIGHT;

    vector<Point2f> fourCnrs{
        Point2f(-l / 2, -w / 2),
        Point2f(l / 2, -w / 2),
        Point2f(l / 2, w / 2),
        Point2f(-l / 2, w / 2)};
    pl_state.cnr.clear();
    pl_state.cnr_x.clear();
    for (int i = 0; i < 4; i++)
    {
        pl_state.cnr.push_back(pl_state.center + fourCnrs[i]);
        Point p = ft.mVision.getPixel(Point3f(pl_state.cnr[i].x, pl_state.cnr[i].y, center_w.z));
        pl_state.cnr_x.push_back(p);
    }
}

void getNormalOfPushLine(FabricState pl_state, double dir[3])
{
    Point2f pl_dir = pl_state.cnr[1] - pl_state.cnr[0];
    pl_dir /= norm(pl_dir);
    dir[0] = -pl_dir.y;
    dir[1] = pl_dir.x;
    dir[2] = 0;
}

// Controller part //
int mouse_control_mode = 0; // 1: waypoint_mode for setting the waypoint of fabric state; 3: user interface for setting pushing line, waypoint and target
bool defined_target = false;
bool t_planned = false;
bool r_planned = false;
bool startSelection = false;
Point mvStart, mvEnd;
Point lineStart, lineEnd;

// p0 and p0_ are corresponding points after rotation
Point2f getRotationCenter(Point2f p0, Point2f p0_, Point2f p1, Point2f p1_)
{
    Point2f m0 = (p0 + p0_) / 2;
    Point2f m1 = (p1 + p1_) / 2;

    double k0 = -(p0.x - p0_.x) / (p0.y - p0_.y);
    double k1 = -(p1.x - p1_.x) / (p1.y - p1_.y);

    double a0 = k0, b0 = -1, c0 = (m0.y - k0 * m0.x);
    double a1 = k1, b1 = -1, c1 = (m1.y - k1 * m1.x);

    Point2f RC((b0 * c1 - b1 * c0) / (a0 * b1 - a1 * b0), (a1 * c0 - a0 * c1) / (a0 * b1 - a1 * b0));
    return RC;
}

bool passThrough(Point p0, Point p1, vector<Point> ctr)
{
    int param_num = 10;
    Point step = (p1 - p0) / param_num;
    for (int i = 0; i <= param_num; i++)
    {
        Point inner_dot = p0 + i * step;
        if (pointPolygonTest(ctr, inner_dot, false) >= 0)
            return true;
    }
    return false;
}

bool fs_start_touched = false;
bool fs_t_touched = false;
Point3f rc_planned;
static void onMouseControl(int event, int x, int y, int flags, void *)
{
    Mat currentFrame;
    imageRGB->image.copyTo(currentFrame);
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
    case EVENT_RBUTTONDOWN:
    case EVENT_MBUTTONDOWN:
        lineStart.x = x;
        lineStart.y = y;
        mvStart.x = x;
        mvStart.y = y;
        // Detect which object is clicked
        // 0 = fs_t.ctr_x | 1 = fs_end.ctr_x | 2 = pushLine_t.cnr_x
        for (int i = 0; i < clickableObjects.size(); i++)
        {
            if (pointPolygonTest(clickableObjects[i]->cnr_x, Point2f(x, y), false) > 0)
            {
                clicked[i] = true;
            }
            else
            {
                clicked[i] = false;
            }
        }

        fs_start_touched = passThrough((pushLine_t.cnr_x[0] + pushLine_t.cnr_x[3]) / 2, (pushLine_t.cnr_x[1] + pushLine_t.cnr_x[2]) / 2, fs_start.ctr_x);
        fs_t_touched = passThrough((pushLine_r.cnr_x[0] + pushLine_r.cnr_x[3]) / 2, (pushLine_r.cnr_x[1] + pushLine_r.cnr_x[2]) / 2, fs_t.ctr_x);
        // record current fabric state
        break;
    case EVENT_LBUTTONUP:
    case EVENT_RBUTTONUP:
    case EVENT_MBUTTONUP:
        mvEnd.x = x;
        mvEnd.y = y;
        lineEnd.x = x;
        lineEnd.y = y;

        for (int i = 0; i < clicked.size(); i++)
        {
            clicked[i] = false;
        }
        break;
    case EVENT_MOUSEMOVE:
        mvEnd.x = x;
        mvEnd.y = y;

        lineEnd.x = x;
        lineEnd.y = y;

        if (mouse_control_mode == 0)
        {
            // translate current fabric contour pixels
            // get the fabric state
        }
        else if (mouse_control_mode == 3 && (flags & EVENT_FLAG_CTRLKEY) && (flags & EVENT_FLAG_LBUTTON) && clicked[2])
        {
            // Define target translation
            defined_target = true;

            // Clear fs_t and fs_r data
            // fs_start.copyTo(fs_t);
            // fs_start.copyTo(fs_r);
            rc_planned = Point3f();

            // define the target state
            // fs_next.copyTo(fs_end);
            Point3f ps = ft.mVision.getWorldCoordinate(mvStart, imageShowDepth);
            Point3f pe = ft.mVision.getWorldCoordinate(mvEnd, imageShowDepth);
            Point2f mv(pe.x - ps.x, pe.y - ps.y);
            ft.transform(fs_end, mv);
        }
        else if (mouse_control_mode == 3 && (flags & EVENT_FLAG_CTRLKEY) && (flags & EVENT_FLAG_RBUTTON) && clicked[2])
        {
            // Define target rotation
            defined_target = true;

            // Clear fs_t and fs_r data
            // fs_start.copyTo(fs_t);
            // fs_start.copyTo(fs_r);
            rc_planned = Point3f();

            double degree = mvEnd.x - mvStart.x;
            double rad = degree * PI / 180;
            ft.transform(fs_end, Point2f(0, 0), fs_end.center, rad);
        }
        else if (mouse_control_mode == 3 && defined_target && fs_start_touched && (flags & EVENT_FLAG_SHIFTKEY) && (flags & EVENT_FLAG_LBUTTON) && clicked[3])
        {
            // T Planning (Pushing line touched first)
            t_planned = true;

            // define the target state
            // fs_next.copyTo(fs_end);
            Point3f ps = ft.mVision.getWorldCoordinate(mvStart, imageShowDepth);
            Point3f pe = ft.mVision.getWorldCoordinate(mvEnd, imageShowDepth);
            Point2f mv(pe.x - ps.x, pe.y - ps.y);
            ft.transform(fs_t, mv);

            // find rotation center of R planning
            Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);
            rc_planned = Point3f(rc.x, rc.y, SURFACE_HEIGHT);

            // fs_t.copyTo(fs_r);

            // DrawFabricFeatures_Path(currentFrame, fs_start, fs_t, 0.05);
        }
        else if (mouse_control_mode == 3 && defined_target && fs_start_touched && (flags & EVENT_FLAG_SHIFTKEY) && (flags & EVENT_FLAG_RBUTTON) && clicked[4])
        {
            // R Planning (Pushing line touched first)
            r_planned = true;

            double degree = mvEnd.x - mvStart.x;
            double rad = degree * PI / 180;
            ft.transform(fs_t, Point2f(0, 0), fs_t.center, rad);

            // find rotation center of R planning
            Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);
            rc_planned = Point3f(rc.x, rc.y, SURFACE_HEIGHT);
        }
        else if (mouse_control_mode == 3 && (flags & EVENT_FLAG_LBUTTON) && (clicked[0] || clicked[1]) && defined_target)
        {
            // Dragging of pushing line (translation)

            // define the target state
            // fs_next.copyTo(fs_end);
            Point3f ps = ft.mVision.getWorldCoordinate(mvStart, imageShowDepth);
            Point3f pe = ft.mVision.getWorldCoordinate(mvEnd, imageShowDepth);
            Point2f mv(pe.x - ps.x, pe.y - ps.y);

            if (clicked[1])
            {
                ft.transform(pushLine_t, mv);
            }
            else if (clicked[0])
            {
                ft.transform(pushLine_r, mv);
            }
        }
        else if (mouse_control_mode == 3 && (flags & EVENT_FLAG_RBUTTON) && (clicked[0] || clicked[1]) && defined_target)
        {
            // Dragging of pushing line (rotation)
            double degree = mvEnd.x - mvStart.x;
            double rad = degree * PI / 180;

            if (clicked[1])
            {
                ft.transform(pushLine_t, Point2f(0, 0), pushLine_t.center, rad);
            }
            else if (clicked[0])
            {
                ft.transform(pushLine_r, Point2f(0, 0), pushLine_r.center, rad);
            }
        }
        else if (mouse_control_mode == 1 && (flags & EVENT_FLAG_CTRLKEY) && (flags & EVENT_FLAG_LBUTTON) && clicked[2])
        {
            // Define target translation
            defined_target = true;

            // Clear fs_t and fs_r data
            // fs_start.copyTo(fs_t);
            // fs_start.copyTo(fs_r);
            rc_planned = Point3f();

            // define the target state
            // fs_next.copyTo(fs_end);
            Point3f ps = ft.mVision.getWorldCoordinate(mvStart, imageShowDepth);
            Point3f pe = ft.mVision.getWorldCoordinate(mvEnd, imageShowDepth);
            Point2f mv(pe.x - ps.x, pe.y - ps.y);
            ft.transform(fs_t, mv);
        }
        else if (mouse_control_mode == 1 && (flags & EVENT_FLAG_CTRLKEY) && (flags & EVENT_FLAG_RBUTTON) && clicked[2])
        {
            // Define target rotation
            defined_target = true;

            // Clear fs_t and fs_r data
            // fs_start.copyTo(fs_t);
            // fs_start.copyTo(fs_r);
            rc_planned = Point3f();

            double degree = mvEnd.x - mvStart.x;
            double rad = degree * PI / 180;
            ft.transform(fs_t, Point2f(0, 0), fs_end.center, rad);
        }

        DrawFabricFeatures_Contour(currentFrame, fs_start.ctr_x, Scalar(0, 255, 0));
        DrawFabricFeatures_Contour(currentFrame, fs_end.ctr_x, Scalar(255, 0, 0));
        DrawFabricFeatures_Contour(currentFrame, fs_t.ctr_x, Scalar(255, 255, 255));
        // DrawFabricFeatures_Contour(currentFrame, fs_r.ctr_x, Scalar(255, 255, 0));
        DrawFabricFeatures_Contour(currentFrame, pushLine_t.cnr_x, Scalar(255, 255, 255));
        DrawFabricFeatures_Contour(currentFrame, pushLine_r.cnr_x, Scalar(255, 0, 0));

        DrawFabricFeatures_RC(currentFrame, rc_planned);

        // if rotation pl touched fs_t, auto plan rotation path from fs_t to fs_end
        if (fs_start_touched)
        {
            t_planned = true;
            DrawTranslationArrow(currentFrame, pushLine_t, fs_t.cnr_x[0] - fs_start.cnr_x[0], Scalar(0, 0, 255));
        }

        if (fs_t_touched)
        {
            r_planned = true;

            // find rotation center of R planning
            Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);
            rc_planned = Point3f(rc.x, rc.y, SURFACE_HEIGHT);

            double rad_r = ft.angleBetween(fs_t.cnr[0] - fs_t.cnr[1], fs_end.cnr[0] - fs_end.cnr[1]);
            DrawRotationArrow(currentFrame, rc_planned, pushLine_r.center, rad_r, Scalar(0, 0, 255));
        }

        imshow("imageRGB_Planner", currentFrame);

        mvStart = mvEnd;

        break;
    }
}

Scalar color_range[9][2]{
    {Scalar(0, 120, 80), Scalar(50, 255, 160)}, // Red
    // {Scalar(0, 90, 120), Scalar(180, 255, 255)},  // old Orange
    {Scalar(0, 100, 160), Scalar(33, 255, 255)},   // Orange  NB_lighting {Scalar(0, 90, 80), Scalar(180, 255, 255)},
    {Scalar(20, 100, 100), Scalar(50, 255, 255)},  // yellow
    {Scalar(50, 60, 100), Scalar(80, 255, 255)},   // green
    {Scalar(80, 50, 120), Scalar(100, 255, 255)},  // blue
    {Scalar(150, 50, 240), Scalar(180, 255, 255)}, // pink
    {Scalar(110, 60, 140), Scalar(140, 255, 255)}, // purple
    {Scalar(0, 0, 0), Scalar(180, 255, 100)},      // black
    {Scalar(0, 0, 100), Scalar(180, 255, 255)}     // white
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

    Mat frame_HSV, mask_f, mask_m;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, color_range[8][0], color_range[8][1], mask_f);
    //inRange(frame_HSV, color_range[1][0], color_range[1][1], mask_m);

    // In mask_f pixel that is outside of rectangle of Point(100, 0), Point(src.cols - 200, src.rows - 50), set them to 0
    for (int i = 0; i < mask_f.rows; i++)
    {
        for (int j = 0; j < mask_f.cols; j++)
        {
            if (i < 0 || i > mask_f.rows - 100 || j < 100 || j > mask_f.cols - 200)
            {
                mask_f.at<uchar>(i, j) = 0;
            }
        }
    }

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
    Mat drawing = Mat::ones(src.size(), CV_8UC3) * 255;
    rectangle(drawing, Point(100, 0), Point(src.cols - 200, src.rows-100), Scalar(0, 0, 0), -1);
    drawContours(drawing, contours, id_max, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point());
    // show drawing
    imshow("drawing", drawing);

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

vector<Point> getMarkerCenter_diffcolors(Mat src, int numOfMarker)
{
    Mat frame_HSV, mask_m;
    cvtColor(src, frame_HSV, COLOR_BGR2HSV);
    vector<Point> pts;

    for (int i = 0; i < numOfMarker; i++)
    {
        inRange(frame_HSV, color_range[i][0], color_range[i][1], mask_m);

        Mat blur, canny_edge;
        medianBlur(mask_m, blur, 3);

        Canny(blur, canny_edge, 20, 140);
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        dilate(canny_edge, canny_edge, element);

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
        // Detect blobs.
        vector<KeyPoint> keypoints;
        detector->detect(canny_edge, keypoints);

        if (keypoints.size() > 0)
        {
            pts.push_back(Point(keypoints[0].pt.x, keypoints[0].pt.y));
        }
        else
        {
            pts.push_back(Point(-1, -1));
        }
    }

    return pts;
}

bool smooth(FabricState fs_c, FabricState fs_req, double allowance, GraspAndMovement &gam, vector<int> &cnr_smoothed, bool first_smooth=false)
{
    cnr_smoothed.clear();
    cnr_smoothed.resize(fs_req.cnr.size(), 0);
    std::vector<CornerAlign> pullToSmooth;
    for (int i = 0; i < fs_req.cnr.size(); i++)
    {
        if (!fs_c.inView[i])
            continue;

        if (fs_req.cnr.size() == 7 && first_smooth && (i == 2 || i == 3 || i == 4))
            continue;

        if (cv::norm(fs_c.cnr[i] - fs_req.cnr[i]) > allowance)
        {
            std::cout << "Corner error of " << i << " is " << cv::norm(fs_c.cnr[i] - fs_req.cnr[i]) << std::endl;
            std::cout << "Corner " << i << " needs to be smoothed!" << std::endl;
            cv::Point2f align_dir = (fs_req.cnr[i] - fs_c.cnr[i]) / cv::norm(fs_req.cnr[i] - fs_c.cnr[i]);

            cv::Point2f leftSideNormal = fs_c.getNormalOfCornerDir(i, "L");
            cv::Point2f rightSideNormal = fs_c.getNormalOfCornerDir(i, "R");

            if (align_dir.dot(leftSideNormal) > 0 || align_dir.dot(rightSideNormal) > 0 || pullToSmooth.empty())
            // if(true)
            {
                std::cout << "Corner " << i << " could be smoothed with its side!" << std::endl;
                CornerAlign ca;
                ca.cnr_idx = i;
                ca.point_from = fs_c.cnr[i];
                ca.point_to = fs_req.cnr[i];

                cv::Point2f pick_normal, place_normal;
                if (align_dir.dot(leftSideNormal) > align_dir.dot(rightSideNormal))
                {
                    ca.side = "L";
                    ca.dir_from = fs_c.cnr_L[i];
                    ca.dir_to = fs_req.cnr_L[i];
                    pick_normal = leftSideNormal;
                    place_normal = fs_req.getNormalOfCornerDir(i, "L");
                }
                else
                {
                    ca.side = "R";
                    ca.dir_from = fs_c.cnr_R[i];
                    ca.dir_to = fs_req.cnr_R[i];
                    pick_normal = rightSideNormal;
                    place_normal = fs_req.getNormalOfCornerDir(i, "R");
                }
                ca.pick_normal = pick_normal;
                ca.place_normal = place_normal;

                cv::Point2f move_dir = (ca.point_to + 0.5 * PL_LENGTH * ca.dir_to) - (ca.point_from + 0.5 * PL_LENGTH * ca.dir_from);
                move_dir /= cv::norm(move_dir);

                if (pick_normal.dot(move_dir) > 0 || pullToSmooth.empty())
                // if(true)
                {
                    ca.move_cos = pick_normal.dot(move_dir);
                    pullToSmooth.push_back(ca);
                }
            }
        }
        else
        {
            cnr_smoothed[i] = 1;
        }
    }

    if (!pullToSmooth.empty())
    {
        // Select the right corner and the right side which can provide a pulling motion for alignment
        // collect a set of possible(move in pulling direction) smoothing candidates
        // choose the candidate whose pulling dir is close to the normal of picking position
        // (min angle has the max cos)
        double max_cos = -1;
        // double max_error = -1;
        double max_ratio = -1;
        int idx = 0;
        for (int i = 0; i < pullToSmooth.size(); i++)
        {

            // if (pullToSmooth[i].move_cos > max_cos)
            double pos_error = cv::norm(pullToSmooth[i].point_to - pullToSmooth[i].point_from);

            // pulling action and with min ratio of translation/rotation
            // angle between dir_from and dir_to
            double ratio = cv::norm(pullToSmooth[i].point_to - pullToSmooth[i].point_from) / acos(pullToSmooth[i].dir_from.dot(pullToSmooth[i].dir_to));
            // if (pullToSmooth[i].move_cos > 0 && pos_error > max_error)
            if (pullToSmooth[i].move_cos > max_cos)
            {
                max_cos = pullToSmooth[i].move_cos;
                // max_error = pos_error;
                // max_ratio = ratio;
                idx = i;
            }
        }

        // (Predefined smoothing position) select the corner with the largest error, assume the i-th corner
        // select the best grasping side, assume the 1st side
        // Parallel Offset the grasping side and the releasing side
        cv::Point2f gp_w = pullToSmooth[idx].point_from + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_from;
        cv::Point2f offset_dir = -fs_c.getNormalOfCornerDir(pullToSmooth[idx].cnr_idx, pullToSmooth[idx].side) + pullToSmooth[idx].dir_from;
        gp_w += PL_OFFSET * offset_dir;

        cv::Point2f ori_l(pullToSmooth[idx].dir_from.y, -pullToSmooth[idx].dir_from.x);
        cv::Point2f ori_r(-pullToSmooth[idx].dir_from.y, pullToSmooth[idx].dir_from.x);
        cv::Point2f gp_ori = pullToSmooth[idx].pick_normal;

        cv::Point2f rp_w = pullToSmooth[idx].point_to + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_to;
        offset_dir = -fs_req.getNormalOfCornerDir(pullToSmooth[idx].cnr_idx, pullToSmooth[idx].side) + pullToSmooth[idx].dir_to;
        rp_w += PL_OFFSET * offset_dir;

        ori_l = cv::Point2f(pullToSmooth[idx].dir_to.y, -pullToSmooth[idx].dir_to.x);
        ori_r = cv::Point2f(-pullToSmooth[idx].dir_to.y, pullToSmooth[idx].dir_to.x);
        cv::Point2f rp_ori = pullToSmooth[idx].place_normal;

        gam.type = 2;
        gam.pick.sticking_point.x = gp_w.x;
        gam.pick.sticking_point.y = gp_w.y;

        gam.pick.sticking_point.z = SURFACE_HEIGHT;

        gam.pick.orientation.x = gp_ori.x;
        gam.pick.orientation.y = gp_ori.y;

        gam.place.sticking_point.x = rp_w.x;
        gam.place.sticking_point.y = rp_w.y;

        gam.place.sticking_point.z = SURFACE_HEIGHT;

        gam.place.orientation.x = rp_ori.x;
        gam.place.orientation.y = rp_ori.y;

        // show the smoothing grasp and motion
        // std::cout << "get position of grasp" << std::endl;
        // cv::Point sp = mVision.getPixel(cv::Point3f(gp_w.x, gp_w.y, fs_c.height));
        // std::cout << "sp Point: " << sp << std::endl;
        // cv::Point sp_d = mVision.getPixel(cv::Point3f(gp_w.x + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_from.x, gp_w.y + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_from.y, fs_c.height));
        // std::cout << "sp dir: " << sp_d << std::endl;

        // cv::Point sp_np = sp;
        // cv::Point sp_nd = mVision.getPixel(cv::Point3f(gp_w.x + 0.02 * gp_ori.x, gp_w.y + 0.02 * gp_ori.y, fs_c.height));

        // std::cout << "get position of release" << std::endl;
        // cv::Point rp = mVision.getPixel(cv::Point3f(rp_w.x, rp_w.y, fs_c.height));
        // cv::Point rp_d = mVision.getPixel(cv::Point3f(rp_w.x + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_to.x, rp_w.y + 0.5 * PL_LENGTH * pullToSmooth[idx].dir_to.y, fs_c.height));

        // cv::Point rp_n = rp;
        // cv::Point rp_nd = mVision.getPixel(cv::Point3f(rp_w.x + 0.02 * rp_ori.x, rp_w.y + 0.02 * rp_ori.y, fs_c.height));

        // std::cout << "draw grasp" << std::endl;
        // cv::line(fabricMat, sp - (sp_d - sp), sp + (sp_d - sp), cv::Scalar(0, 0, 255), 2);
        // cv::line(fabricMat, rp - (rp_d - rp), rp + (rp_d - rp), cv::Scalar(0, 255, 0), 2);
        // cv::arrowedLine(fabricMat, sp, rp, cv::Scalar(0, 255, 0), 2);
        // std::cout << "------------------- 1.6 --------------" << std::endl;
        // cv::arrowedLine(fabricMat, sp_np, sp_nd, cv::Scalar(255, 255, 255), 2);
        // cv::arrowedLine(fabricMat, rp_n, rp_nd, cv::Scalar(255, 255, 255), 2);

        // std::vector<std::vector<cv::Point>> ctr_set;
        // ctr_set.push_back(fs_req.ctr_x);
        // cv::drawContours(fabricMat, ctr_set, 0, cv::Scalar(0, 255, 255));
        // cv::imshow("smoothing_grasp", fabricMat);

        return true;
    }
    else
    {
        return false;
    }
}

vector<geometry_msgs::Point> point2msg(vector<Point> data)
{
    vector<geometry_msgs::Point> to;
    for (int i = 0; i < data.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = data[i].x;
        p.y = data[i].y;
        to.push_back(p);
    }

    return to;
}

vector<int8_t> int2msg(vector<int> data)
{
    vector<int8_t> to;
    for (int i = 0; i < data.size(); i++)
    {
        to.push_back(data[i]);
    }

    return to;
}

vector<Point> get_sm_lines(GraspAndMovement gam)
{
    vector<Point> sm_lines;
    Point2f dir_sp(-gam.pick.orientation.y, gam.pick.orientation.x);
    cv::Point sp = ft.mVision.getPixel(cv::Point3f(gam.pick.sticking_point.x, gam.pick.sticking_point.y, SURFACE_HEIGHT));
    cv::Point sp_d = ft.mVision.getPixel(cv::Point3f(gam.pick.sticking_point.x + 0.5 * PL_LENGTH * dir_sp.x, gam.pick.sticking_point.y + 0.5 * PL_LENGTH * dir_sp.y, SURFACE_HEIGHT));

    Point2f dir_rp(-gam.place.orientation.y, gam.place.orientation.x);
    cv::Point rp = ft.mVision.getPixel(cv::Point3f(gam.place.sticking_point.x, gam.place.sticking_point.y, SURFACE_HEIGHT));
    cv::Point rp_d = ft.mVision.getPixel(cv::Point3f(gam.place.sticking_point.x + 0.5 * PL_LENGTH * dir_rp.x, gam.place.sticking_point.y + 0.5 * PL_LENGTH * dir_rp.y, SURFACE_HEIGHT));

    sm_lines.push_back(sp - (sp_d - sp));
    sm_lines.push_back(sp + (sp_d - sp));
    sm_lines.push_back(rp - (rp_d - rp));
    sm_lines.push_back(rp + (rp_d - rp));

    return sm_lines;
}

double get_avg_error(vector<Point2f> curr, vector<Point2f> target)
{
    double error_tot = 0;
    for (int i = 0; i < curr.size(); i++)
    {
        error_tot += norm(curr[i] - target[i]);
    }
    return error_tot / curr.size();
}

FabricState get_state_from_camera(int num_marker)
{
    // get 10 frame from camera and average them
    ros::Rate rate(30);

    // create a vector of Mat to store 10 frames
    vector<Mat> imageDepths;

    for (int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        rate.sleep();

        while (!received)
        {
            cout << "Didn't received image data!" << endl;
            ros::spinOnce();
            rate.sleep();
        }
        if (received)
        {
            cout << "Received image data!" << endl;
            imageDepths.push_back(imageDepth->image);
            received = false;
        }
    }
    // average 10 frames
    Mat avgDepth = Mat::zeros(imageDepth->image.size(), CV_16UC1);
    for (int i = 0; i < imageDepths.size(); i++)
    {
        avgDepth += imageDepths[i];
    }
    avgDepth /= imageDepths.size();


    avgDepth.copyTo(imageShowDepth);
    imageRGB->image.copyTo(imageShowRGB);
    

    vector<Point> fabric_outline = getFabricContour(imageShowRGB);
    // // show fabric outline
    // cv::Mat fabricMat = cv::Mat::zeros(imageShowRGB.size(), CV_8UC3);
    // vector<vector<Point>> ctr_set;
    // ctr_set.push_back(fabric_outline);
    // cv::drawContours(fabricMat, ctr_set, 0, cv::Scalar(0, 255, 255));
    // cv::imshow("fabricMat", fabricMat);
    // cv::waitKey(0);

    vector<Point> markers = getMarkerCenter(imageShowRGB);
    if (get_marker_from_topic)
    {
        markers = cnr_marker;
    }
    else
    {
        markers = getMarkerCenter(imageShowRGB);
    }
    // vector<Point> markers = getMarkerCenter_diffcolors(imageShowRGB, num_marker);

    // // show markers
    // cv::Mat markerMat = cv::Mat::zeros(imageShowRGB.size(), CV_8UC3);
    // for (int i = 0; i < markers.size(); i++)
    // {
    //     cv::circle(markerMat, markers[i], 5, cv::Scalar(0, 0, 255), -1);
    //     cv::putText(markerMat, to_string(i), markers[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    // }
    // cv::imshow("markerMat", markerMat);
    // cv::waitKey(0);

    FabricState newState = ft.getFabricStateFromData(markers, fabric_outline, imageShowDepth, PL_LENGTH, get_marker_from_topic);
    return newState;
}

// value 0...1
double find_IOU(vector<Point> c1, vector<Point> c2, Size s)
{
    cv::Mat mask1 = cv::Mat::zeros(s.height, s.width, CV_8UC1);
    cv::Mat mask2 = cv::Mat::zeros(s.height, s.width, CV_8UC1);

    vector<vector<Point>> ctr_set;
    ctr_set.push_back(c1);
    ctr_set.push_back(c2);

    drawContours(mask1, ctr_set, 0, 255, -1);
    drawContours(mask2, ctr_set, 1, 255, -1);

    cv::Mat intersectArea = (mask1 & mask2); // binary image with only with blob 1
    // cv::Mat intersectArea;
    // bitwise_and(mask1, mask2, intersectArea);
    cv::Mat unionArea = (mask1 | mask2); // binary image with only with blob 2

    cv::Scalar sum1 = cv::sum(intersectArea);
    cv::Scalar sum2 = cv::sum(unionArea);

    double ratio = sum1[0] / sum2[0];

    return ratio;
}

void marker_callback(const std_msgs::Float64MultiArray &msg)
{
    cnr_marker.clear();
    for (int i = 0; i < msg.data.size() / 2; i++)
    {
        cnr_marker.push_back(cv::Point(msg.data[2 * i], msg.data[2 * i + 1]));
    }
}

// 0: no primary motion
void reposition2pos(Manipulator manipulator, FabricState fs_to, int num_marker, ros::ServiceClient uiService, bool uiUpdate = true, bool primary_mode = 0)
{
    bool first_action = true;
    // 3. Smoothing
    bool canSmooth = true;
    while (canSmooth)
    {

        FabricState fs_c = get_state_from_camera(num_marker);

        GraspAndMovement gam;
        vector<int> cnr_smoothed;

        if(first_action){
            canSmooth = smooth(fs_c, fs_to, ALLOWANCE, gam, cnr_smoothed, true);
            first_action = false;
        }else{
            canSmooth = smooth(fs_c, fs_to, ALLOWANCE, gam, cnr_smoothed);
        }
        

        if (canSmooth)
        {
            if (uiUpdate)
            {
                /* Update UI for rotation */
                soft_align::UpdateUI uiSrv;
                // uiSrv.request.fs_start = point2msg(fs_start.ctr_x);
                uiSrv.request.fs_end = point2msg(fs_to.ctr_x);
                // msg of target position and matched boolean
                uiSrv.request.cnr_t = point2msg(fs_to.cnr_x);
                uiSrv.request.matched_t = int2msg(cnr_smoothed);

                // uiSrv.request.fs_c = point2msg(fs_c.ctr_x);

                // Smooth
                // uiSrv.request.sm_lines = point2msg(get_sm_lines(gam));
                // //  Update text UI
                // string str = "PC: " + to_string(PL_times) + " | " + "ACE: " + to_string(avg_errors[PL_times]) + " | " + "IOU: " + to_string(iOUs[PL_times]);
                // uiSrv.request.description = str;

                if (uiService.call(uiSrv))
                {
                    std::cout << "Successful to update rotation UI" << std::endl;
                }
                else
                {
                    std::cout << "Failed to update rotation UI" << std::endl;
                }
            }

            // Do smoothing control
            double pos_s[3] = {gam.pick.sticking_point.x, gam.pick.sticking_point.y, gam.pick.sticking_point.z + STICKING_HIGHT - GRIPPER_OFFSET};
            double dir_s[3] = {gam.pick.orientation.x, gam.pick.orientation.y, 0};
            cout << "Place the push line with the required position and orientation." << endl;
            manipulator.grasp(dir_s, pos_s);
            double pos_rls[3] = {gam.place.sticking_point.x, gam.place.sticking_point.y, gam.place.sticking_point.z + STICKING_HIGHT - GRIPPER_OFFSET};
            double dir_rls[3] = {gam.place.orientation.x, gam.place.orientation.y, 0};
            manipulator.release(dir_rls, pos_rls);
            // PL_times++;

            manipulator.moveUp(0.04);
            manipulator.ready();
            // fs_curr = get_state_from_camera(num_marker);
            // avg_errors.push_back(get_avg_error(fs_end.cnr, fs_curr.cnr));
            // iOUs.push_back(find_IOU(fs_end.ctr_x, fs_curr.ctr_x, imageShowRGB.size()));
        }
        else
        {
            if (uiUpdate)
            {
                soft_align::UpdateUI uiSrv;
                uiSrv.request.fs_end = point2msg(fs_to.ctr_x);
                uiSrv.request.cnr_t = point2msg(fs_to.cnr_x);
                uiSrv.request.matched_t = int2msg(cnr_smoothed);

                if (uiService.call(uiSrv))
                {
                    std::cout << "Successful to update rotation UI" << std::endl;
                }
                else
                {
                    std::cout << "Failed to update rotation UI" << std::endl;
                }
            }
        }
    }
}

int main(int argc, char **argv)
{

    int num_marker = atoi(argv[2]);

    int PL_times = 0;
    vector<double> avg_errors; // len(avg_errors) == PL_times+1
    vector<double> iOUs;

    if (argc < 3)
    {
        std::cout << "Not enough arguments passed in!" << std::endl;
        exit(-1);
    }

    int mode = 1; // 0: manual plan; 1: auto R plan; 2: auto TR plan

    string pattern = argv[1];

    ros::init(argc, argv, "soft_align_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::Rate rate(30);

    message_filters::Subscriber<sensor_msgs::Image> imageRGB_sub(node, "/camera0/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> imageDepth_sub(node, "/camera0/aligned_depth_to_color/image_raw", 1);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imageRGB_sub, imageDepth_sub);
    sync.registerCallback(boost::bind(&chatterCallbackCVC, _1, _2));
    ros::ServiceClient uiClient = node.serviceClient<soft_align::UpdateUI>("/soft_align/update_ui");

    ros::Subscriber marker_sub = node.subscribe("/soft_align/mapped_markers", 1, marker_callback);

    double move_x, move_y, theta;
    if (mode != 0)
    {
        // get the transformation input from msg, format: [x,y,theta]
        boost::shared_ptr<std_msgs::Float64MultiArray const> t_msg;
        t_msg = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/fabric_alignment/transformation", ros::Duration(5));

        move_x = t_msg->data[0];
        move_y = t_msg->data[1];
        theta = t_msg->data[2];
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    namedWindow("imageRGB_Planner");
    setMouseCallback("imageRGB_Planner", onMouseControl, 0);

    /* Init robot */
    MoveitCommander robot;
    Manipulator manipulator(&robot);

    manipulator.ready();

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
    ft.mVision = mVision;
    string tp_file = pattern + ".txt";
    ft.initTP(tp_file);

    /* Get the Fabric current state */
    fs_start = get_state_from_camera(num_marker);
    fs_start.copyTo(fs_end);
    fs_start.copyTo(fs_t);
    // fs_t.copyTo(fs_r);

    // Set the target state
    double rad = theta * PI / 180;
    ft.transform(fs_end, Point2f(move_x, move_y), fs_end.center, rad);

    // Init pushline state
    initPushLine(pushLine_t, PL_LENGTH, PL_WIDTH, plt_pos);
    initPushLine(pushLine_r, PL_LENGTH, PL_WIDTH, plr_pos);

    // Tracking all FabricState objects in a set, the order determines the tracking priority when being clicked.
    clickableObjects.push_back(&pushLine_r);
    clickableObjects.push_back(&pushLine_t);
    clickableObjects.push_back(&fs_end);
    clickableObjects.push_back(&fs_t);
    clickableObjects.push_back(&fs_t);

    clicked = vector<bool>(clickableObjects.size(), false);

    /* User input terminal */
    // int method;
    // cout << "Input the planning method (0:T-R | 1:TR): ";
    // cin >> method;

    soft_align::UpdateUI uiInitSrv;
    if (uiClient.call(uiInitSrv))
    {
        std::cout << "Successful to update translation UI" << std::endl;
    }
    else
    {
        std::cout << "Failed to update translation UI" << std::endl;
    }

    // manipulator.calibrate();

    // **** Start of primary action **** //

    if (use_primary)
    {

        // A. Init the target position, and do the planning
        if (mode == 0)
        {
            // Manual mode
            mouse_control_mode = 3;
            int key = 0;
            imshow("imageRGB_Planner", imageShowRGB);
            // Break loop if space key is pressed, to quit the imshow
            while (true)
            {
                int key = waitKey(0);
                if (key == 32)
                    break;
            }
        }
        else if (mode == 1)
        {
            // Auto R planning

            // Audo plan the PL randomly above the center of fs_start
            // Point2f pl_move = fs_start.center - pushLine_r.center;
            // ft.transform(pushLine_r, pl_move);

            // Option 2: plan PL on the side that it's close to the target position.
            double minDist = 999;
            int minIdx = 0;
            for (int i = 0; i < fs_start.cnr.size(); i++)
            {
                double dist = norm(fs_end.center - fs_start.cnr[i]);
                if (dist < minDist)
                {
                    minDist = dist;
                    minIdx = i;
                }
            }

            std::cout << "---------minIdx-----------: " << minIdx << std::endl;

            // calculate the transformation of pushLine_r to the side
            Point2f side_dir;
            Point2f mv_dir = fs_end.center - fs_start.cnr[minIdx];
            mv_dir /= norm(mv_dir);

            if (mv_dir.dot(fs_start.cnr_L[minIdx]) > mv_dir.dot(fs_start.cnr_R[minIdx]))
            {
                side_dir = fs_start.cnr_L[minIdx];
            }
            else
            {
                side_dir = fs_start.cnr_R[minIdx];
            }

            Point2f side_normal(side_dir.y, -side_dir.x);

            Point2f pl_center = fs_start.cnr[minIdx] + 0.5 * PL_LENGTH * side_dir;

            Point2f pl_dir = pushLine_r.cnr[0] - pushLine_r.cnr[3];
            pl_dir /= norm(pl_dir);

            double rot = ft.angleBetween(pl_dir, side_normal);
            ft.transform(pushLine_r, pl_center - pushLine_r.center, pushLine_r.center, rot);
        }
        else if (mode == 2)
        {
            // Auto TR planning
            rad = theta * PI / 180;
            Point2f fcenter = fs_end.center;
            ft.transform(fs_end, Point2f(move_x, move_y), fs_end.center, rad);

            // cout << "move command: " << Point2f(move_x, move_y) << endl;
            // cout << "movement of transform function: " << fs_end.center - fcenter << endl;

            // // Audo plan the PL randomly above the center of fs_start
            // Point2f pl_move = fs_start.center - pushLine_r.center;
            // ft.transform(pushLine_r, pl_move);
            // cout << "PL c: " << pushLine_r.center << endl;
            // cout << "fs_start.center: " << fs_start.center << endl;

            // Option 2: plan PL on the side that it's close to the target position.
            double minDist = 999;
            int minIdx = 0;
            for (int i = 0; i < fs_start.cnr.size(); i++)
            {
                double dist = norm(fs_end.center - fs_start.cnr[i]);
                if (dist < minDist)
                {
                    minDist = dist;
                    minIdx = i;
                }
            }

            // Option 3: plan PL on the side that the moving direction is to the outside of contour

            // calculate the transformation of pushLine_r to the side
            Point2f side_dir;
            Point2f mv_dir = fs_end.center - fs_start.cnr[minIdx];
            mv_dir /= norm(mv_dir);

            if (mv_dir.dot(fs_start.cnr_L[minIdx]) > mv_dir.dot(fs_start.cnr_R[minIdx]))
            {
                side_dir = fs_start.cnr_L[minIdx];
            }
            else
            {
                side_dir = fs_start.cnr_R[minIdx];
            }

            Point2f side_normal(side_dir.y, -side_dir.x);

            Point2f pl_center = fs_start.cnr[minIdx] + 0.5 * PL_LENGTH * side_dir;

            Point2f pl_dir = pushLine_r.cnr[0] - pushLine_r.cnr[3];
            pl_dir /= norm(pl_dir);

            double rot = ft.angleBetween(pl_dir, side_normal);
            ft.transform(pushLine_r, pl_center - pushLine_r.center, pushLine_r.center, rot);
        }

        // start error
        avg_errors.push_back(get_avg_error(fs_end.cnr, fs_start.cnr));
        iOUs.push_back(find_IOU(fs_end.ctr_x, fs_start.ctr_x, imageShowRGB.size()));

        // Start timer
        // Recording the timestamp at the start of the code
        auto beg = high_resolution_clock::now();

        // 1. Translation
        // params: PL: position, orientation; Action: movement
        if (t_planned)
        {
            /* Update UI for translation */
            soft_align::UpdateUI uiSrv;
            uiSrv.request.fs_start = point2msg(fs_start.ctr_x);
            uiSrv.request.fs_end = point2msg(fs_end.ctr_x);
            uiSrv.request.cnr_t = point2msg(fs_end.cnr_x);

            // init the matched_t to all zeros with size of fs_end.cnr_x
            vector<int> cnr_smoothed(fs_end.cnr_x.size(), 0);
            uiSrv.request.matched_t = int2msg(cnr_smoothed);

            uiSrv.request.pl_t = point2msg(pushLine_t.cnr_x);
            uiSrv.request.fs_t = point2msg(fs_t.ctr_x);
            //  Update text UI
            string str = "PC: " + to_string(PL_times) + " | " + "ACE: " + to_string(avg_errors[PL_times]) + " | " + "IOU: " + to_string(iOUs[PL_times]);
            uiSrv.request.description = str;

            // line arrow
            vector<Point> arrow;
            Point pl_center = (pushLine_t.cnr_x[0] + pushLine_t.cnr_x[1] + pushLine_t.cnr_x[2] + pushLine_t.cnr_x[3]) / 4;
            Point t_end = pl_center + (fs_t.cnr_x[0] - fs_start.cnr_x[0]);
            arrow.push_back(pl_center);
            arrow.push_back(t_end);
            uiSrv.request.arrowedline_t = point2msg(arrow);

            if (uiClient.call(uiSrv))
            {
                std::cout << "Successful to update translation UI" << std::endl;
            }
            else
            {
                std::cout << "Failed to update translation UI" << std::endl;
            }

            double pos_t[3] = {pushLine_t.center.x, pushLine_t.center.y, pushLine_t.height + STICKING_HIGHT - GRIPPER_OFFSET};
            double dir_t[3];
            getNormalOfPushLine(pushLine_t, dir_t);

            cout << "Place the push line with the required position and orientation." << endl;
            manipulator.grasp(dir_t, pos_t);

            cout << "Translate the pattern by pushing." << endl;
            Point2f mv = fs_t.cnr[0] - fs_start.cnr[0];
            double movement[3] = {mv.x, mv.y, 0};
            manipulator.translate(movement);
            PL_times++;

            robot.moveXYZ(0, 0, 0.04);
            manipulator.ready();

            fs_curr = get_state_from_camera(num_marker);
            avg_errors.push_back(get_avg_error(fs_end.cnr, fs_curr.cnr));
            iOUs.push_back(find_IOU(fs_end.ctr_x, fs_curr.ctr_x, imageShowRGB.size()));
        }

        // 2. Rotation (Primary action)
        /* Update UI for rotation */
        soft_align::UpdateUI uiSrv;
        uiSrv.request.fs_start = point2msg(fs_start.ctr_x);
        uiSrv.request.fs_end = point2msg(fs_end.ctr_x);
        uiSrv.request.fs_t = point2msg(fs_t.ctr_x);
        uiSrv.request.pl_r = point2msg(pushLine_r.cnr_x);
        uiSrv.request.cnr_t = point2msg(fs_end.cnr_x);
        // init the matched_t to all zeros with size of fs_end.cnr_x
        vector<int> cnr_smoothed(fs_end.cnr_x.size(), 0);
        uiSrv.request.matched_t = int2msg(cnr_smoothed);

        // //  Update text UI
        string str = "PC: " + to_string(PL_times) + " | " + "ACE: " + to_string(avg_errors[PL_times]) + " | " + "IOU: " + to_string(iOUs[PL_times]);
        // uiSrv.request.description = str;

        if (mode == 1)
        {
            // R planning
            // curve arrow
            double rad_curve = ft.angleBetween(fs_t.cnr[0] - fs_t.cnr[1], fs_end.cnr[0] - fs_end.cnr[1]);

            // find rotation center of R planning
            if (theta != 0)
            {
                Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);
                rc_planned = Point3f(rc.x, rc.y, SURFACE_HEIGHT);
                vector<Point> pointList_x = dotOnCurve(rc_planned, pushLine_r.center, rad_curve, 10);
                uiSrv.request.arrowedcurve_r = point2msg(pointList_x);
            }
            else
            {
                vector<Point> arrow;
                Point pl_center = (pushLine_r.cnr_x[0] + pushLine_r.cnr_x[1] + pushLine_r.cnr_x[2] + pushLine_r.cnr_x[3]) / 4;
                arrow.push_back(pl_center);
                arrow.push_back(ft.mVision.getPixel(Point3f(fs_end.center.x, fs_end.center.y, SURFACE_HEIGHT)));
                uiSrv.request.arrowedline_t = point2msg(arrow);
            }
        }
        else if (mode == 2)
        {
            // TR planning
            // line arrow
            vector<Point> arrow;
            double rad_curve = ft.angleBetween(fs_t.cnr[0] - fs_t.cnr[1], fs_end.cnr[0] - fs_end.cnr[1]);
            // find rotation center of R planning
            if (theta != 0)
            {
                Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);
                rc_planned = Point3f(rc.x, rc.y, SURFACE_HEIGHT);
                vector<Point> pointList_x = dotOnCurve(rc_planned, pushLine_r.center, rad_curve, 2);
                Point pl_center = (pushLine_r.cnr_x[0] + pushLine_r.cnr_x[1] + pushLine_r.cnr_x[2] + pushLine_r.cnr_x[3]) / 4;
                arrow.push_back(pl_center);
                arrow.push_back(pointList_x[pointList_x.size() - 1]);
                uiSrv.request.arrowedcurve_r = point2msg(arrow);
            }
            else
            {
                Point pl_center = (pushLine_r.cnr_x[0] + pushLine_r.cnr_x[1] + pushLine_r.cnr_x[2] + pushLine_r.cnr_x[3]) / 4;
                arrow.push_back(pl_center);
                arrow.push_back(ft.mVision.getPixel(Point3f(pushLine_r.center.x + move_x, pushLine_r.center.y + move_y, SURFACE_HEIGHT)));
                uiSrv.request.arrowedline_t = point2msg(arrow);
            }
        }

        if (uiClient.call(uiSrv))
        {
            std::cout << "Successful to update rotation UI" << std::endl;
        }
        else
        {
            std::cout << "Failed to update rotation UI" << std::endl;
        }

        // params: PL: position, orientation; Action: center, angle
        double pos_r[3] = {pushLine_r.center.x, pushLine_r.center.y, pushLine_r.height + STICKING_HIGHT - GRIPPER_OFFSET};
        double dir_r[3];
        getNormalOfPushLine(pushLine_r, dir_r);

        cout << "Place the push line with the required position and orientation." << endl;
        manipulator.grasp(dir_r, pos_r);

        Point2f rc(0, 0);
        int numofp = fs_t.cnr.size();
        for (int i = 0; i < fs_t.cnr.size(); i++)
        {
            Point2f rc_0 = getRotationCenter(fs_t.cnr[i], fs_end.cnr[i], fs_t.cnr[(i + 1) % fs_t.cnr.size()], fs_end.cnr[(i + 1) % fs_t.cnr.size()]);
            rc += rc_0;
        }
        rc /= numofp;

        // Point2f rc = getRotationCenter(fs_t.cnr[0], fs_end.cnr[0], fs_t.cnr[1], fs_end.cnr[1]);

        double rad_r = ft.angleBetween(fs_t.cnr[0] - fs_t.cnr[1], fs_end.cnr[0] - fs_end.cnr[1]);
        double center[2] = {rc.x, rc.y};

        cout << "Rotate the pattern by pushing." << endl;
        cout << "rotation angle by setting(real): " << rad << endl;
        cout << "rotation angle by calculation: " << rad_r << endl;

        cout << "rotation center-fabric's CM (real): " << fs_end.center << endl;
        cout << "rotation center by calculation: " << rc << endl;

        // check if rc is nan
        if (theta == 0 || rc.x != rc.x)
        {
            std::cout << "rc is nan" << std::endl;
            // fs_end.center - pushLine_r.center
            // Point2f mv(move_x, move_y);
            Point2f mv = fs_end.cnr[0] - fs_start.cnr[0];
            double movement[3] = {mv.x, mv.y, 0};
            manipulator.translate(movement);
        }
        else
        {
            std::cout << "rc is not nan" << std::endl;
            if (mode == 1)
                manipulator.rotateR(center, rad_r);
            else if (mode == 2)
                manipulator.rotateTR(center, rad_r);
        }

        // PL_times++;

        robot.moveXYZ(0, 0, 0.04);
        manipulator.ready();
        // fs_curr = get_state_from_camera();
        // avg_errors.push_back(get_avg_error(fs_end.cnr, fs_curr.cnr));
        // iOUs.push_back(find_IOU(fs_end.ctr_x, fs_curr.ctr_x, imageShowRGB.size()));
    }
    // **** End of primary action **** //

    // Set the pose of waypoint
    // if (waypoint_mode)
    // {
    //     mouse_control_mode = 1;
    //     int key = 0;
    //     imshow("imageRGB_Planner", imageShowRGB);
    //     // Break loop if space key is pressed, to quit the imshow
    //     while (true)
    //     {
    //         int key = waitKey(0);
    //         if (key == 32)
    //             break;
    //     }
    // }

    // FabricState fs_to;
    // fs_end.copyTo(fs_to);

    /* Update UI to initilized target position */
    soft_align::UpdateUI finalSrv;
    // uiSrv.request.fs_start = point2msg(fs_start.ctr_x);
    finalSrv.request.fs_end = point2msg(fs_end.ctr_x);
    finalSrv.request.cnr_t = point2msg(fs_end.cnr_x);
    // initilize a vector of size 4 to -1
    vector<int> cnr_smoothed(num_marker, 0);
    finalSrv.request.matched_t = int2msg(cnr_smoothed);

    if (uiClient.call(finalSrv))
    {
        std::cout << "Successful to update rotation UI" << std::endl;
    }
    else
    {
        std::cout << "Failed to update rotation UI" << std::endl;
    }

    // reposition2pos(manipulator, fs_t, num_marker, uiClient, true);
    reposition2pos(manipulator, fs_end, num_marker, uiClient, true);

    

    // uiSrv.request.fs_c = point2msg(fs_c.ctr_x);

    // Taking a timestamp after the code is ran
    // auto end = high_resolution_clock::now();
    // auto duration = duration_cast<seconds>(end - beg);

    //  Update text UI
    // str = "PC: " + to_string(PL_times) + " | " + "ACE: " + to_string(avg_errors[PL_times]) + " | " + "IOU: " + to_string(iOUs[PL_times]) + " | Time: " + to_string(duration.count()) + "s";
    // finalSrv.request.description = str;
    // if (uiClient.call(finalSrv))
    // {
    //     std::cout << "Successful to update rotation UI" << std::endl;
    // }
    // else
    // {
    //     std::cout << "Failed to update rotation UI" << std::endl;
    // }

    // if(mode == 1)
    //     cout << "Planning mode: " << "R Planning" << endl;
    // else if(mode == 2)
    //     cout << "Planning mode: " << "TR Planning" << endl;

    // cout << "Transformation: " << "[" << move_x << "," << move_y << "," << theta << "]" << endl;
    // std::cout << "Elapsed Time: " << to_string(duration.count()) << "s" << endl;
    // cout << "PC: " << PL_times << endl;
    // cout << "Average error of corner:" << endl;
    // for (int i = 0; i < avg_errors.size(); i++)
    // {
    //     cout << avg_errors[i] << endl;
    // }
    // cout << "IOU: " << endl;
    // for (int i = 0; i < iOUs.size(); i++)
    // {
    //     cout << iOUs[i] << endl;
    // }

    return 0;
}