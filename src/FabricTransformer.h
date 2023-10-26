#ifndef FABRICTRANSFORMER_H
#define FABRICTRANSFORMER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/types.hpp>
#include <algorithm>
#include "new_type.h"
#include "Vision.h"
#include "FabricState.h"

using namespace std;
using namespace cv;

#define PI 3.141592653
#define THRES_NEAR 10

vector<vector<int>> allmaps;
std::vector<cv::Point> selected_cnrs;

class FabricTransformer
{
public:
    // std::vector<cv::Point> fabricContour_I;
    // std::vector<cv::Point> fabricCorner_I;

    double allowance;
    bool initialized;
    cv::Point2f center;
    std::string tp_file;
    std::vector<cv::Point2f> tp_cnrs;
    std::vector<double> segments_tp, inner_corners_tp;

    // for recording index of ordered corner on contour
    std::vector<int> cnr_idx_on_ctr;

    Vision mVision;

    FabricState state_c;  // current state
    FabricState state_cr; // current required state
    FabricState state_nr; // next required state
    FabricState state_t;  // target state

    bool flattened = true;

    FabricTransformer() {}

    FabricTransformer(std::vector<cv::Point> fabricContour_I)
    {
        // get fabricCorner_C and fabricCornerDirection_C from the fabricContour_I
        // cv::Point3f pt3f = getPointInCameraFrame(cnt[0][i], imageShowDepth, &camera_intrin_rgb);
        // cv::Point3f wp = cameraToWorld(pt3f);
        initialized = false;
    }

    void initVision(realsense_ros::camera_intrin ci, tf::TransformListener *tfl, std::string name)
    {
        mVision = Vision(ci, tfl, name);
    }

    void initTP(std::string file)
    {
        tp_file = file;

        // read template(eg. p1.x p1.y p2.x p2.y  ...)
        std::string data_path = ros::package::getPath("soft_object_tracking") + "/data/";
        std::ifstream tp_data(data_path + tp_file);
        cv::Point2f cnr;
        while (tp_data >> cnr.x)
        {
            tp_data >> cnr.y;
            tp_cnrs.push_back(cnr);
            std::cout << cnr.x << " " << cnr.y << std::endl;
        }
        tp_data.close();

        int mod = tp_cnrs.size();
        for (int i = 0; i < tp_cnrs.size(); i++)
        {
            int last = ((i - 1) % mod + mod) % mod;
            int next = (i + 1) % mod;
            segments_tp.push_back(cv::norm(tp_cnrs[next] - tp_cnrs[i]));
            inner_corners_tp.push_back(angleBetween(tp_cnrs[next] - tp_cnrs[i], tp_cnrs[last] - tp_cnrs[i]));
        }
    }

    // ref and markers should have the same size
    double computeCost(std::vector<cv::Point2f> ref, std::vector<cv::Point2f> markers, double w1, double w2)
    {

        int mod = markers.size();
        std::vector<double> segments, inner_corners;
        std::vector<double> segments_tp, inner_corners_tp;
        for (int i = 0; i < markers.size(); i++)
        {
            int last = ((i - 1) % mod + mod) % mod;
            int next = (i + 1) % mod;
            segments.push_back(cv::norm(markers[next] - markers[i]));
            inner_corners.push_back(angleBetween(markers[next] - markers[i], markers[last] - markers[i]));

            segments_tp.push_back(cv::norm(ref[next] - ref[i]));
            inner_corners_tp.push_back(angleBetween(ref[next] - ref[i], ref[last] - ref[i]));
        }

        double seg_ct = 0;
        double ang_ct = 0;

        for (int j = 0; j < markers.size(); j++)
        {
            // segments_tp[j] compared with segments[(i+j)%mod]

            seg_ct += pow(segments[j] - segments_tp[j], 2) / segments_tp[j];
            ang_ct += pow(inner_corners[j] - inner_corners_tp[j], 2) / abs(inner_corners_tp[j]);
        }
        double cost = w1 * seg_ct + w2 * ang_ct;

        return cost;
    }

    // a function to get the index of cloest point on contour to the given point
    int getClosestIdx(cv::Point p, vector<cv::Point> ctr)
    {
        int idx = 0;
        double min_dist = 100000;
        for (int i = 0; i < ctr.size(); i++)
        {
            double dist = cv::norm(p - ctr[i]);
            if (dist < min_dist)
            {
                min_dist = dist;
                idx = i;
            }
        }
        return idx;
    }

    void updateCornerByMarker(std::vector<cv::Point> markers, std::vector<cv::Point> contour, std::vector<cv::Point> &cnr_x, cv::Mat &depth_img)
    {

        // if cnr_idx_on_ctr is empty, then initialize it
        // else, update the corner

        std::vector<int> cnr_idx_on_ctr_new(cnr_x.size(), -1);

        // record idx of marker that is assigned to the corner, -1 means no marker is assigned to it in this frame
        std::vector<int> cnr_mkr_idx(cnr_x.size(), -1);

        // A vector stores bool value, true means marker has been assigned to a corner
        std::vector<bool> mkr_assigned_flag(markers.size(), false);

        // first, assign markers to corners
        for (int i = 0; i < markers.size(); i++)
        {
            double min_dist = 100000;
            int min_id = -1;
            for (int j = 0; j < cnr_x.size(); j++)
            {

                double dist = cv::norm(markers[i] - cnr_x[j]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_id = j;
                }
            }
            if (min_id != -1)
            {
                if (cnr_mkr_idx[min_id] == -1)
                {

                    cnr_mkr_idx[min_id] = i;
                    cnr_x[min_id] = markers[i];
                    mkr_assigned_flag[i] = true;

                    // // update the corner idx on contour
                    // int closest_idx = getClosestIdx(markers[i], contour);
                    // cnr_idx_on_ctr_new[min_id] = closest_idx;
                }
                else
                {
                    // compare the distance between the marker and the corner
                    double dist1 = cv::norm(markers[i] - cnr_x[min_id]);
                    double dist2 = cv::norm(markers[cnr_mkr_idx[min_id]] - cnr_x[min_id]);
                    if (dist1 < dist2)
                    {
                        mkr_assigned_flag[cnr_mkr_idx[min_id]] = false;
                        cnr_mkr_idx[min_id] = i;
                        cnr_x[min_id] = markers[i];
                        mkr_assigned_flag[i] = true;

                        // // update the corner idx on contour
                        // int closest_idx = getClosestIdx(markers[i], contour);
                        // cnr_idx_on_ctr_new[min_id] = closest_idx;
                    }
                }
            }
        }

        // // if there is no marker assigned to any corner, then map the marker to the corner
        // getCornersFromMarkers(markers, contour, depth_img, cnr_x, inView)

        std::vector<cv::Point> compared_markers;
        std::vector<int> compared_markers_idx;

        // construct a vector of corners that have been assigned to a marker
        // and a vector of the corresponding corners idx
        for (int i = 0; i < cnr_mkr_idx.size(); i++)
        {
            if (cnr_mkr_idx[i] != -1)
            {
                compared_markers.push_back(cnr_x[i]);
                compared_markers_idx.push_back(i);
            }
        }

        // assign the rest of the markers to the corners that has no marker assigned to it
        for (int i = 0; i < mkr_assigned_flag.size(); i++)
        {
            if (mkr_assigned_flag[i] == false)
            {
                double minCost = 100000;
                int min_id = -1;

                for (int j = 0; j < cnr_x.size(); j++)
                {
                    if (cnr_mkr_idx[j] == -1)
                    {
                        std::vector<cv::Point> compared_markers_candidate = compared_markers;
                        std::vector<int> compared_markers_idx_candidate = compared_markers_idx;
                        // find the place in the compared_markers_idx_candidate that j is between
                        int idx = 0;
                        for (int k = 0; k < compared_markers_idx_candidate.size(); k++)
                        {
                            if (j > compared_markers_idx_candidate[k])
                            {
                                idx = k;
                            }
                        }
                        compared_markers_candidate.insert(compared_markers_candidate.begin() + idx, markers[i]);
                        compared_markers_idx_candidate.insert(compared_markers_idx_candidate.begin() + idx, j);

                        double cost = matchCost(compared_markers_candidate, compared_markers_idx_candidate, depth_img);
                        if (cost < minCost)
                        {
                            minCost = cost;
                            min_id = j;
                        }
                    }
                }
                if (min_id != -1)
                {
                    cnr_mkr_idx[min_id] = i;
                    cnr_x[min_id] = markers[i];
                    mkr_assigned_flag[i] = true;
                }
            }
        }

        // update corners that lost markers to -1 point
        // for (int i = 0; i < cnr_mkr_idx.size(); i++)
        // {
        //     if (cnr_mkr_idx[i] == -1)
        //     {
        //         cnr_x[i] = cv::Point(-1, -1);
        //     }
        // }
    }

    // Generate the next required state
    void transform(FabricState &fs, cv::Point2f t, cv::Point2f center = cv::Point2f(0, 0), double rad = 0)
    {
        // rotation plus translation with coordinate frame cetered at CM(center of mass) of fabric
        double R[2][2] = {
            {cos(rad), -sin(rad)},
            {sin(rad), cos(rad)}};

        if (center == cv::Point2f(0, 0))
        {
            center = fs.center;
        }

        // transform fabric contour
        for (int i = 0; i < fs.ctr.size(); i++)
        {
            cv::Point2f cp = fs.ctr[i] - center;
            cv::Point2f rp(R[0][0] * cp.x + R[0][1] * cp.y, R[1][0] * cp.x + R[1][1] * cp.y);
            cv::Point2f tp = rp + center;
            fs.ctr[i] = tp + t;
        }
        // transform fabric corners
        for (int i = 0; i < fs.cnr.size(); i++)
        {

            cv::Point2f cp = fs.cnr[i] - center;
            cv::Point2f rp(R[0][0] * cp.x + R[0][1] * cp.y, R[1][0] * cp.x + R[1][1] * cp.y);
            cv::Point2f tp = rp + center;
            fs.cnr[i] = tp + t;
        }
        // transform fabric corner direction
        for (int i = 0; i < fs.cnr_L.size(); i++)
        {

            cv::Point2f newDir;
            newDir.x = R[0][0] * fs.cnr_L[i].x + R[0][1] * fs.cnr_L[i].y;
            newDir.y = R[1][0] * fs.cnr_L[i].x + R[1][1] * fs.cnr_L[i].y;
            fs.cnr_L[i] = newDir;
            newDir.x = R[0][0] * fs.cnr_R[i].x + R[0][1] * fs.cnr_R[i].y;
            newDir.y = R[1][0] * fs.cnr_R[i].x + R[1][1] * fs.cnr_R[i].y;
            fs.cnr_R[i] = newDir;
        }
        // update required positions in pixel
        for (int i = 0; i < fs.ctr.size(); i++)
        {
            cv::Point3f wp(fs.ctr[i].x, fs.ctr[i].y, fs.height);
            cv::Point3f cwp = mVision.worldToCamera(wp);
            cv::Point pp = mVision.project_point_to_pixel(cwp);
            fs.ctr_x[i] = pp;
        }
        for (int i = 0; i < fs.cnr.size(); i++)
        {
            cv::Point3f wp(fs.cnr[i].x, fs.cnr[i].y, fs.height);
            cv::Point3f cwp = mVision.worldToCamera(wp);
            cv::Point pp = mVision.project_point_to_pixel(cwp);
            fs.cnr_x[i] = pp;
        }

        // update center of new fabric state
        cv::Point2f com(0, 0);
        for (int i = 0; i < fs.cnr.size(); i++)
        {
            com += fs.cnr[i];
        }
        int size = fs.cnr.size();
        fs.center = com / size;
        fs.ori += rad;
    }

    void updateState(FabricState &fs, std::string name)
    {
        if (name == "C")
        {
            fs.copyTo(state_c);
        }
        else if (name == "R")
        {
            fs.copyTo(state_cr);
        }
        else if (name == "T")
        {
            fs.copyTo(state_t);
        }
    }

    // Input is the fabric contour, output is the main corners of the contour
    std::vector<cv::Point> getCorners(std::vector<cv::Point> contour)
    {
        double cnt_len = cv::arcLength(contour, true);
        std::vector<cv::Point> cnt;
        cv::approxPolyDP(contour, cnt, 0.02 * cnt_len, true);

        return cnt;
    }

    std::vector<cv::Point> getConvexCorners(std::vector<cv::Point> contour)
    {
        double cnt_len = cv::arcLength(contour, true);
        std::vector<cv::Point> cnt, convexCnt;
        cv::approxPolyDP(contour, cnt, 0.02 * cnt_len, true);

        std::cout << "Coor of cnr: " << std::endl;
        for (int i = 0; i < convexCnt.size(); i++)
            std::cout << cnt[i] << std::endl;

        cv::convexHull(cnt, convexCnt);
        std::cout << "Coor of convex cnr: " << std::endl;
        for (int i = 0; i < convexCnt.size(); i++)
            std::cout << convexCnt[i] << std::endl;

        return convexCnt;
    }

    std::vector<cv::Point> getCornersManually(cv::Mat &src, cv::Mat &imageShowDepth)
    {
        // clear global saver
        selected_cnrs.clear();

        cv::imshow("corner_selector", src);
        cv::waitKey(0);

        // save corner angles and segments to the template file
        // std::vector<double> segments_tp, inner_corners_tp;
        std::vector<cv::Point2f> selected_cnrs_w;
        for (int i = 0; i < selected_cnrs.size(); i++)
        {
            cv::Point3f pt3 = mVision.getWorldCoordinate(selected_cnrs[i], imageShowDepth);
            cv::Point2f pt2(pt3.x, pt3.y);
            selected_cnrs_w.push_back(pt2);
        }

        // int mod = selected_cnrs.size();
        // for (int i = 0; i < selected_cnrs.size(); i++)
        // {
        //     int last = ((i - 1) % mod + mod) % mod;
        //     int next = (i + 1) % mod;
        //     segments_tp.push_back(cv::norm(selected_cnrs_w[next] - selected_cnrs_w[i]));
        //     inner_corners_tp.push_back(angleBetween(selected_cnrs_w[next] - selected_cnrs_w[i], selected_cnrs_w[last] - selected_cnrs_w[i]));
        // }

        std::string data_path = ros::package::getPath("soft_object_tracking") + "/data/";
        std::ofstream tp_data;
        tp_data.open(data_path + "shorts.txt");
        for (int i = 0; i < selected_cnrs.size(); i++)
            tp_data << selected_cnrs_w[i].x << ' ' << selected_cnrs_w[i].y << ' ';
        // tp_data << '\n';

        tp_data.close();

        return selected_cnrs;
    }

    void selectCornersAndSave(cv::Mat &src, cv::Mat &imageShowDepth, std::string file)
    {
        // clear global saver
        selected_cnrs.clear();

        cv::imshow("corner_selector", src);
        cv::waitKey(0);

        // save corner angles and segments to the template file
        // std::vector<double> segments_tp, inner_corners_tp;
        std::vector<cv::Point2f> selected_cnrs_w;

        for (int i = 0; i < selected_cnrs.size(); i++)
        {
            cv::Point3f pt3 = mVision.getWorldCoordinate(selected_cnrs[i], imageShowDepth);
            cv::Point2f pt2(pt3.x, pt3.y);
            selected_cnrs_w.push_back(pt2);
        }

        // int mod = selected_cnrs.size();
        // for (int i = 0; i < selected_cnrs.size(); i++)
        // {
        //     int last = ((i - 1) % mod + mod) % mod;
        //     int next = (i + 1) % mod;
        //     segments_tp.push_back(cv::norm(selected_cnrs_w[next] - selected_cnrs_w[i]));
        //     inner_corners_tp.push_back(angleBetween(selected_cnrs_w[next] - selected_cnrs_w[i], selected_cnrs_w[last] - selected_cnrs_w[i]));
        // }
        std::string data_path = ros::package::getPath("soft_object_tracking") + "/data/";
        std::ofstream tp_data;
        tp_data.open(data_path + file);
        for (int i = 0; i < selected_cnrs.size(); i++)
            tp_data << selected_cnrs_w[i].x << ' ' << selected_cnrs_w[i].y << ' ';
        // tp_data << '\n';
        tp_data.close();
    }

    double angleBetween(cv::Point2f v1, cv::Point2f v2)
    {
        // double len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
        // double len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

        // double dot = v1.x * v2.x + v1.y * v2.y;

        // double a = dot / (len1 * len2);

        // double sign = v1.x * v2.y - v1.y * v2.x;

        // if (a >= 1.0)
        //     return 0.0;
        // else if (a <= -1.0)
        //     return PI;
        // else
        // {
        //     if (sign > 0)
        //         return acos(a); // 0..PI
        //     else
        //         return -acos(a);
        // }

        double dot_product = v1.x * v2.x + v1.y * v2.y;
        double det = v1.x * v2.y - v1.y * v2.x;
        double angle = atan2(det, dot_product);

        return angle;
    }

    void getInvisibleCorners(std::vector<cv::Point> &convexCnt, std::vector<bool> &inView, cv::Mat &imageShowDepth)
    {
        // find the index of first and second visible corner
        int first = -1, second = -1;
        for (int i = 0; i < convexCnt.size(); i++)
        {
            if (inView[i])
            {
                if (first == -1)
                    first = i;
                else
                {
                    second = i;
                    break;
                }
            }
        }

        cv::Point3f pt0 = mVision.getWorldCoordinate(convexCnt[first], imageShowDepth);
        cv::Point3f pt1 = mVision.getWorldCoordinate(convexCnt[second], imageShowDepth);
        cv::Point2f v2(pt1.x - pt0.x, pt1.y - pt0.y);
        cv::Point2f v1 = tp_cnrs[second] - tp_cnrs[first];

        double angle;
        double c = (v1.x * v2.x + v1.y * v2.y) / (cv::norm(v1) * cv::norm(v2));
        double z = v1.x * v2.y - v2.x * v1.y;
        if (z >= 0)
        {
            angle = acos(c);
        }
        else
        {
            angle = -acos(c);
        }

        std::vector<cv::Point2f> vertices;
        for (int i = 0; i < tp_cnrs.size(); i++)
        {
            cv::Point2f v_std = tp_cnrs[i] - tp_cnrs[first];
            // rotate
            cv::Point2f v_std_r;
            v_std_r.x = v_std.x * cos(angle) - v_std.y * sin(angle);
            v_std_r.y = v_std.x * sin(angle) + v_std.y * cos(angle);
            vertices.push_back(v_std_r);
        }

        for (int i = 0; i < tp_cnrs.size(); i++)
        {
            if (!inView[i])
            {
                cv::Point2f p_est(pt0.x + vertices[i].x, pt0.y + vertices[i].y);

                cv::Point3f wp(p_est.x, p_est.y, pt0.z);
                cv::Point3f cwp = mVision.worldToCamera(wp);
                cv::Point pp = mVision.project_point_to_pixel(cwp);

                convexCnt[i] = pp;
            }
        }

    }

    // map the corner with the template, corners should be fully detected with the same size as the that of template.
    // return the marker centers in the same order of the template
    void getCornersFromMarkers(std::vector<cv::Point> marker_centers, std::vector<cv::Point> fabric_outline, cv::Mat &imageShowDepth, std::vector<cv::Point> &corners, std::vector<bool> &inView, bool mapped = false)
    {
        std::cout << "Marker size: " << marker_centers.size() << std::endl;
        std::cout << "Outline size: " << fabric_outline.size() << std::endl;
        // std::cout << "Depth size: " << imageShowDepth.size() << std::endl;

        // template matching to index the marker_centers corresponding to corners of the template
        std::vector<cv::Point> matchedCorners;
        std::vector<cv::Point> convexCnt;
        std::vector<int> order;
        if (!mapped)
        {
            std::cout << "----------------- !mapped -----------------" << std::endl;
            convexCnt = orderByContour(marker_centers, fabric_outline);
            order = map(convexCnt, imageShowDepth);
            // for
        }
        else
        {
            for (int i = 0; i < marker_centers.size(); i++)
            {
                order.push_back(i);
            }
        }

        // print order
        // std::cout << "order: ";
        // for (int i = 0; i < order.size(); i++)
        //     std::cout << order[i] << ' ';
        // std::cout << std::endl;

        // // show order on the image, and ask user to confirm or reinput the order
        // // create a zero image with the same size of the depth image
        // cv::Mat orderImg = cv::Mat::zeros(imageShowDepth.size(), CV_8UC1);
        // // draw markers on the image
        // for (int i = 0; i < marker_centers.size(); i++)
        // {
        //     cv::circle(orderImg, convexCnt[i], 5, cv::Scalar(255), -1);
        //     // label the order with number from order
        //     cv::putText(orderImg, std::to_string(order[i]), convexCnt[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);
        // }
        // cv::imshow("order", orderImg);
        // cv::waitKey(0);

        // // receive input from user, if input is empty, then use the default order, else parse the input eg. 1,2,3,4
        // std::string input;
        // std::cout << "Please input the order of the corners, eg. 1,2,3,4" << std::endl;
        // std::getline(std::cin, input);
        // if (input.empty())
        // {
        //     std::cout << "Use default order" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Use input order" << std::endl;
        //     std::vector<int> input_order;
        //     std::stringstream ss(input);
        //     int i;
        //     while (ss >> i)
        //     {
        //         input_order.push_back(i);
        //         if (ss.peek() == ',')
        //             ss.ignore();
        //     }
        //     order = input_order;
        // }

        // at least 4 points
        // at least 2 points in view
        if (marker_centers.size() >= 2)
        {
            // std::vector<cv::Point2f> src, dst;
            // for (int i = 0; i < 4; i++)
            // {
            //     cv::Point3f pt3 = mVision.getWorldCoordinate(convexCnt[i], imageShowDepth);
            //     cv::Point2f pt2(pt3.x, pt3.y);
            //     h_tot += pt3.z;
            //     dst.push_back(pt2);
            //     src.push_back(tp_cnrs[order[i]]);
            // }
            // height = h_tot/4;
            // cv::Mat tf_matrix = cv::getPerspectiveTransform(src, dst);

            // when some points are missing
            std::vector<int>::iterator it;
            inView.clear();

            // assume at least two kps are found
            // find the rotation angle
            // find two points in marker_centers that is not (-1,-1)
            // int i = 0;
            // int j = 0;
            // while (i < marker_centers.size() && marker_centers[i].x == -1)
            //     i++;
            // j = i + 1;
            // while (j < marker_centers.size() && marker_centers[j].x == -1)
            //     j++;
            // std::cout << "i: " << i << std::endl;

            cv::Point3f pt0 = mVision.getWorldCoordinate(convexCnt[0], imageShowDepth);
            cv::Point3f pt1 = mVision.getWorldCoordinate(convexCnt[1], imageShowDepth);
            cv::Point2f v2(pt1.x - pt0.x, pt1.y - pt0.y);
            cv::Point2f v1 = tp_cnrs[order[1]] - tp_cnrs[order[0]];

            double angle;
            double c = (v1.x * v2.x + v1.y * v2.y) / (cv::norm(v1) * cv::norm(v2));
            double z = v1.x * v2.y - v2.x * v1.y;
            if (z >= 0)
            {
                angle = acos(c);
            }
            else
            {
                angle = -acos(c);
            }

            std::vector<cv::Point2f> vertices;
            for (int i = 0; i < tp_cnrs.size(); i++)
            {
                cv::Point2f v_std = tp_cnrs[i] - tp_cnrs[order[0]];
                // rotate
                cv::Point2f v_std_r;
                v_std_r.x = v_std.x * cos(angle) - v_std.y * sin(angle);
                v_std_r.y = v_std.x * sin(angle) + v_std.y * cos(angle);
                vertices.push_back(v_std_r);
            }

            for (int i = 0; i < tp_cnrs.size(); i++)
            {
                it = std::find(order.begin(), order.end(), i);
                if (it != order.end())
                // if(marker_centers[i].x != -1 && marker_centers[i].y != -1)
                {
                    matchedCorners.push_back(convexCnt[it - order.begin()]);
                    // matchedCorners.push_back(convexCnt[i]);
                    inView.push_back(true);
                }
                else
                {
                    cv::Point2f p_est(pt0.x + vertices[i].x, pt0.y + vertices[i].y);
                    // can't find it in the convexCnt(viewed corner list)
                    // std::vector<cv::Point2f> in, out;
                    // in.push_back(tp_cnrs[i]);
                    // cv::perspectiveTransform(in, out, tf_matrix);
                    // convert world coordinates to pixels
                    cv::Point3f wp(p_est.x, p_est.y, pt0.z);
                    cv::Point3f cwp = mVision.worldToCamera(wp);
                    cv::Point pp = mVision.project_point_to_pixel(cwp);

                    matchedCorners.push_back(pp);
                    inView.push_back(false);
                }

                // find the index of point on fabric_outline that is closest to matchedCorners[-1]
                int idx = 0;
                double min_dist = 100000;
                for (int j = 0; j < fabric_outline.size(); j++)
                {
                    double dist = cv::norm(fabric_outline[j] - matchedCorners.back());
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        idx = j;
                    }
                }
                cnr_idx_on_ctr.push_back(idx);
            }
        }

        // for the missing keypoints, recover its location.
        // find transformation matrix
        // label the estimated kp as missing state

        // int mod = order.size();

        // for (int j = 0; j < order.size(); j++)
        // {
        //     int it = (i + j) % mod;
        //     matchedCorners.push_back(convexCnt[it]);
        // }

        corners = matchedCorners;
    }

    // void combinationUtil(int *arr, int n, int r,
    //                      int index, int data[], int i);
    // The main function that prints all
    // combinations of size r in arr[]
    // of size n. This function mainly
    // uses combinationUtil()
    void collectCombination(int arr[], int n, int r)
    {
        // A temporary array to store
        // all combination one by one
        int data[r];

        // Print all combination using
        // temporary array 'data[]'
        combinationUtil(arr, n, r, 0, data, 0);
    }

    /* arr[] ---> Input Array
    n ---> Size of input array
    r ---> Size of a combination to be printed
    index ---> Current index in data[]
    data[] ---> Temporary array to store current combination
    i ---> index of current element in arr[] */
    void combinationUtil(int arr[], int n, int r,
                         int index, int data[], int i)
    {
        // Current combination is ready, print it
        if (index == r)
        {
            // cycle permutation
            for (int fst = 0; fst < r; fst++)
            {
                vector<int> a_map;
                for (int j = 0; j < r; j++)
                {
                    // cout << " ("<< data[(fst+j)%r].x << ", " << data[(fst+j)%r].y << ") ";
                    a_map.push_back(data[(fst + j) % r]);
                }
                allmaps.push_back(a_map);
            }
            // cout << endl;
            return;
        }

        // When no more elements are there to put in data[]
        if (i >= n)
            return;

        // current is included, put next at next location
        data[index] = arr[i];
        combinationUtil(arr, n, r, index + 1, data, i + 1);

        // current is excluded, replace it with next (Note that
        // i+1 is passed, but index is not changed)
        combinationUtil(arr, n, r, index, data, i + 1);
    }

    // mapping the corner points to the template, can handle the partial mapping when it's under occlusion
    // return the mapped indices in the template.
    std::vector<int> map(std::vector<cv::Point> kps, cv::Mat &imageShowDepth)
    {
        std::vector<cv::Point2f> markers;

        for (int i = 0; i < kps.size(); i++)
        {
            cv::Point3f pt3f = mVision.getWorldCoordinate(kps[i], imageShowDepth);
            markers.push_back(cv::Point2f(pt3f.x, pt3f.y));
        }
        int n = tp_cnrs.size();

        int arr[20]; // assume keypoint size will not exceed 20

        for (int i = 0; i < n; i++)
            arr[i] = i;

        int r = kps.size();

        allmaps.clear();
        collectCombination(arr, n, r);

        int minIdx = 0;
        double minCost = 99999;
        // all combination of maps are stored in allmaps
        for (int i = 0; i < allmaps.size(); i++)
        {
            std::vector<cv::Point2f> ref;
            for (int j = 0; j < allmaps[i].size(); j++)
            {
                ref.push_back(tp_cnrs[allmaps[i][j]]);
            }
            double cost = computeCost(ref, markers, 1, 1.0);
            if (cost < minCost)
            {
                minCost = cost;
                minIdx = i;
            }
        }

        return allmaps[minIdx];
    }

    // compute the cost between markers with the template slicing by idx
    // return the cost
    double matchCost(std::vector<cv::Point> markers, std::vector<int> idx, cv::Mat &imageShowDepth)
    {
        std::vector<cv::Point2f> ref;
        for (int i = 0; i < idx.size(); i++)
        {
            ref.push_back(tp_cnrs[idx[i]]);
        }
        std::vector<cv::Point2f> markers_w;

        for (int i = 0; i < markers.size(); i++)
        {
            cv::Point3f pt3f = mVision.getWorldCoordinate(markers[i], imageShowDepth);
            markers_w.push_back(cv::Point2f(pt3f.x, pt3f.y));
        }

        return computeCost(ref, markers_w, 1, 1.0);
    }

    int getIndex(vector<int> v, int K)
    {
        auto it = find(v.begin(), v.end(), K);

        // If element was found
        if (it != v.end())
        {

            // calculating the index
            // of K
            int index = it - v.begin();
            return index;
        }
        else
        {
            // If the element is not
            // present in the vector
            return -1;
        }
    }

    std::vector<cv::Point> orderByContour(std::vector<cv::Point> markers, std::vector<cv::Point> fabric_outline)
    {
        std::vector<cv::Point> out;
        std::vector<int> indices;
        for (int i = 0; i < markers.size(); i++)
        {
            int idx = closest_idx(fabric_outline, markers[i]);
            indices.push_back(idx);
        }
        std::vector<int> order = indices;
        std::sort(order.begin(), order.end());
        for (int i = 0; i < order.size(); i++)
        {
            int idx = getIndex(indices, order[i]);
            out.push_back(markers[idx]);
        }

        return out;
    }

    bool allInView()
    {
        for (int i = 0; i < state_c.cnr.size(); i++)
        {
            if (!state_c.inView[i])
                return false;
        }
        return true;
    }

    // only check visible corners
    bool matched(double allowance)
    {
        for (int i = 0; i < state_c.cnr.size(); i++)
        {
            if (state_c.inView[i] && cv::norm(state_c.cnr[i] - state_cr.cnr[i]) > allowance)
                return false;
        }
        return true;
    }

    // void updateFabricPosition(std::vector<cv::Point> fabricContour_I){
    //     std::vector<cv::Point> cnrs = getCorners(fabricContour_I);

    // }

    int closest_idx(std::vector<cv::Point> contour, cv::Point point)
    {
        int smallest_dist = 9999;
        int s_idx = 0;
        for (int i = 0; i < contour.size(); i++)
        {
            int dist = cv::norm(contour[i] - point);
            if (dist < smallest_dist)
            {
                s_idx = i;
                smallest_dist = dist;
            }
        }
        return s_idx;
    }

    std::vector<cv::Point> curveToFit(std::vector<cv::Point> contour, std::vector<cv::Point> corners, int start, int end)
    {
        std::cout << "Contour size: " << contour.size() << std::endl;
        std::cout << "Start id: " << start << std::endl;
        std::cout << "End id: " << end << std::endl;
        int p1 = closest_idx(contour, corners[start]);
        int p2 = closest_idx(contour, corners[end]);
        std::cout << "Two corners' coordinate: " << corners[start] << ", " << corners[end] << std::endl;
        std::cout << "Corresponding contour idx: " << p1 << ", " << p2 << std::endl;
        std::vector<cv::Point> curve;
        int x1, x2;
        if (p1 > p2)
        {
            x1 = p2;
            x2 = p1;
        }
        else
        {
            x1 = p1;
            x2 = p2;
        }
        if ((x2 - x1) < x1 + (contour.size() - x2))
        {
            for (int i = x1; i <= x2; i++)
                curve.push_back(contour[i]);
        }
        else
        {
            for (int i = x2; i < contour.size(); i++)
            {
                curve.push_back(contour[i]);
            }
            for (int i = 0; i <= x1; i++)
            {
                curve.push_back(contour[i]);
            }
        }

        if (cv::norm(corners[start] - curve[0]) < cv::norm(corners[end] - curve[0]))
            return curve;
        else
        {
            std::reverse(curve.begin(), curve.end());
            return curve;
        }
    }

    std::vector<cv::Point> findGraspedCurve(std::vector<cv::Point> curve, double grasp_len, cv::Mat &imageShowDepth)
    {
        // double side_len = cv::norm(cornerPoints[guildingSideIdx[0]] - cornerPoints[guildingSideIdx[1]]);
        std::vector<cv::Point> grasp_curve;

        cv::Point3f endA = mVision.getWorldCoordinate(curve[0], imageShowDepth);
        for (int i = 1; i < curve.size(); i++)
        {

            cv::Point3f endB = mVision.getWorldCoordinate(curve[i], imageShowDepth);
            if (cv::norm(endA - endB) < grasp_len)
            {
                grasp_curve.push_back(curve[i]);
            }
            else
            {
                break;
            }
        }

        return grasp_curve;
    }

    cv::Point2f getDirToEndBOnCurve(std::vector<cv::Point> curve, std::vector<cv::Point> cnrs, int idx_l, int idx_r, double grasp_length, cv::Mat &imageShowDepth)
    {
        // the grasp curve starts from idx_l to idx_r
        // std::cout << "----- 041 0 -------" << std::endl;
        // std::vector<cv::Point> curve = curveToFit(fabric_contour, cnrs, idx_l, idx_r);

        std::vector<cv::Point> grasp_curve = findGraspedCurve(curve, grasp_length, imageShowDepth);
        // The grasp dir
        // startP, endP
        // Sticking pos (gc = grasp center)
        cv::Point3f endA = mVision.getWorldCoordinate(grasp_curve[0], imageShowDepth);
        cv::Point3f endB = mVision.getWorldCoordinate(grasp_curve[grasp_curve.size() - 1], imageShowDepth);
        cv::Point2f dir(endB.x - endA.x, endB.y - endA.y);
        dir /= cv::norm(dir);

        return dir;
    }

    cv::Point2f getDirToEndB(std::vector<cv::Point> cnrs, int idx_l, int idx_r, double grasp_length, cv::Mat &imageShowDepth)
    {
        // the grasp curve starts from idx_l to idx_r
        // std::cout << "----- 041 0 -------" << std::endl;
        // std::vector<cv::Point> curve = curveToFit(fabric_contour, cnrs, idx_l, idx_r);

        // Sticking pos (gc = grasp center)
        cv::Point3f endA = mVision.getWorldCoordinate(cnrs[idx_l], imageShowDepth);
        cv::Point3f endB = mVision.getWorldCoordinate(cnrs[idx_r], imageShowDepth);
        cv::Point2f dir(endB.x - endA.x, endB.y - endA.y);
        dir /= cv::norm(dir);

        return dir;
    }

    // l should be smaller than r
    bool containOthers(std::vector<int> idxs, int l, int r)
    {
        for (int i = 0; i < idxs.size(); i++)
        {
            if (idxs[i] > l && idxs[i] < r)
                return true;
        }

        return false;
    }

    FabricState getFabricStateFromData(std::vector<cv::Point> marker_centers, std::vector<cv::Point> fabricContour, cv::Mat image_depth, double grasp_length, bool cnr_mapped = false)
    {
        FabricState fs;
        std::vector<cv::Point> cnrs;
        std::vector<bool> inView;
        if (cnr_mapped)
        {
            cnrs = marker_centers;
            inView = std::vector<bool>(marker_centers.size(), true);
            for(int i = 0; i < marker_centers.size(); i++)
            {
                if(cnrs[i] == cv::Point(0, 0))
                    inView[i] = false;
            }
            getInvisibleCorners(cnrs, inView, image_depth);
        }
        else
        {
            getCornersFromMarkers(marker_centers, fabricContour, image_depth, cnrs, inView);
        }

        // print cnrs
        for (int i = 0; i < cnrs.size(); i++)
        {
            std::cout << "cnrs: " << cnrs[i] << std::endl;
        }
        std::cout << "cnrs size: " << cnrs.size() << std::endl;

        // // show cnrs on image
        // cv::Mat imageShow = cv::Mat::zeros(image_depth.size(), CV_8UC3);
        // for (int i = 0; i < cnrs.size(); i++)
        // {
        //     cv::circle(imageShow, cnrs[i], 5, cv::Scalar(0, 0, 255), -1);
        //     // label
        //     std::string label = std::to_string(i);
        //     cv::putText(imageShow, label, cnrs[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        // }
        // cv::imshow("cnrs", imageShow);
        // cv::waitKey(0);

        // print inView
        for (int i = 0; i < inView.size(); i++)
        {
            std::cout << "inView: " << inView[i] << std::endl;
        }
        std::cout << "inView size: " << inView.size() << std::endl;

        std::cout << "Get corners from markers: OK!" << std::endl;
        fs.inView = inView;
        fs.cnr.clear();
        fs.cnr_x.clear();
        fs.cnr.resize(cnrs.size());
        fs.cnr_x.resize(cnrs.size());

        std::cout << " ----------- 0  -------------" << std::endl;

        cv::Point2f center(0, 0);
        double height = 0;
        for (int i = 0; i < cnrs.size(); i++)
        {
            cv::Point3f pt3f = mVision.getWorldCoordinate(cnrs[i], image_depth);
            fs.cnr[i] = cv::Point2f(pt3f.x, pt3f.y);
            fs.cnr_x[i] = cnrs[i];

            center += fs.cnr[i];
            height += pt3f.z;
        }
        std::cout << " ----------- 1  -------------" << std::endl;
        fs.ctr.clear();
        fs.ctr_x.clear();
        fs.ctr.resize(fabricContour.size());
        fs.ctr_x.resize(fabricContour.size());
        for (int i = 0; i < fs.ctr.size(); i++)
        {
            cv::Point3f pt3f = mVision.getWorldCoordinate(fabricContour[i], image_depth);
            fs.ctr[i] = cv::Point2f(pt3f.x, pt3f.y);
            fs.ctr_x[i] = fabricContour[i];
        }
        std::cout << " ----------- 2  -------------" << std::endl;
        int size = fs.cnr.size();
        fs.center = center / size;
        fs.height = height / size;
        fs.cnr_L.clear();
        fs.cnr_L.resize(cnrs.size());
        fs.cnr_R.clear();
        fs.cnr_R.resize(cnrs.size());
        int mod = cnrs.size();

        // index of corners in contour list
        std::vector<int> cnr_idx;
        for (int i = 0; i < cnrs.size(); i++)
        {
            int idx = closest_idx(fabricContour, cnrs[i]);
            cnr_idx.push_back(idx);
        }
        std::cout << " ----------- 3  -------------" << std::endl;
        // curve of each side
        std::vector<std::vector<cv::Point>> curves_L, curves_R;
        int mod_cnr = cnrs.size();
        int mod_ctr = fabricContour.size();
        for (int i = 0; i < cnrs.size(); i++)
        {
            std::vector<cv::Point> cvs_l, cvs_r;
            int idx = cnr_idx[i];
            int idx_L = cnr_idx[((i - 1) % mod_cnr + mod_cnr) % mod_cnr];
            int idx_R = cnr_idx[((i + 1) % mod_cnr + mod_cnr) % mod_cnr];
            // Left curve
            if (idx < idx_L && !containOthers(cnr_idx, idx, idx_L))
            {
                int s = idx;
                while (s != idx_L)
                {
                    cvs_l.push_back(fabricContour[s]);
                    s++;
                }
            }
            else if (idx < idx_L && containOthers(cnr_idx, idx, idx_L))
            {
                int s = idx;
                while (s != idx_L)
                {
                    cvs_l.push_back(fabricContour[s]);
                    s = ((s - 1) % mod_ctr + mod_ctr) % mod_ctr;
                }
            }
            else if (idx > idx_L && !containOthers(cnr_idx, idx_L, idx))
            {
                int s = idx_L;
                while (s != idx)
                {
                    cvs_l.push_back(fabricContour[s]);
                    s++;
                }
                // reverse the list
                reverse(cvs_l.begin(), cvs_l.end());
            }
            else if (idx > idx_L && containOthers(cnr_idx, idx_L, idx))
            {
                int s = idx_L;
                while (s != idx)
                {
                    cvs_l.push_back(fabricContour[s]);
                    s = ((s - 1) % mod_ctr + mod_ctr) % mod_ctr;
                }
                // reverse the list
                reverse(cvs_l.begin(), cvs_l.end());
            }
            curves_L.push_back(cvs_l);
            // Right curve
            if (idx < idx_R && !containOthers(cnr_idx, idx, idx_R))
            {
                int s = idx;
                while (s != idx_R)
                {
                    cvs_r.push_back(fabricContour[s]);
                    s++;
                }
            }
            else if (idx < idx_R && containOthers(cnr_idx, idx, idx_R))
            {
                int s = idx;
                while (s != idx_R)
                {
                    cvs_r.push_back(fabricContour[s]);
                    s = ((s - 1) % mod_ctr + mod_ctr) % mod_ctr;
                }
            }
            else if (idx > idx_R && !containOthers(cnr_idx, idx_R, idx))
            {
                int s = idx_R;
                while (s != idx)
                {
                    cvs_r.push_back(fabricContour[s]);
                    s++;
                }
                // reverse the list
                reverse(cvs_r.begin(), cvs_r.end());
            }
            else if (idx > idx_R && containOthers(cnr_idx, idx_R, idx))
            {
                int s = idx_R;
                while (s != idx)
                {
                    cvs_r.push_back(fabricContour[s]);
                    s = ((s - 1) % mod_ctr + mod_ctr) % mod_ctr;
                }
                // reverse the list
                reverse(cvs_r.begin(), cvs_r.end());
            }
            curves_R.push_back(cvs_r);
        }
        std::cout << " ----------- 4  -------------" << std::endl;
        // divide the contour by the corner points into segments
        for (int i = 0; i < cnrs.size(); i++)
        {
            // adjust the corner direction so that the grasping points on the linear gripper can touch the boundary of fabric
            int il = ((i - 1) % mod + mod) % mod;
            int ir = ((i + 1) % mod + mod) % mod;
            if (fs.inView[i])
            {
                fs.cnr_L[i] = getDirToEndB(cnrs, i, il, grasp_length, image_depth);
                fs.cnr_R[i] = getDirToEndB(cnrs, i, ir, grasp_length, image_depth);
            }
            else
            {
                fs.cnr_L[i] = getDirToEndB(fs.cnr_x, i, il, grasp_length, image_depth);
                fs.cnr_R[i] = getDirToEndB(fs.cnr_x, i, ir, grasp_length, image_depth);
            }
        }

        // get the orientation of the fabric, use arctan2
        cv::Point2f dir = fs.cnr[1] - fs.cnr[2];
        fs.ori = atan2(dir.y, dir.x);
        

        return fs;
    }

    void updateCorners(std::vector<cv::Point> fabricContour_I, int ref_id, cv::Mat &imageShowRGB, cv::Mat &image_depth, double grasp_length)
    {
        std::vector<cv::Point> cnrs = getCornersManually(imageShowRGB, image_depth);
        std::vector<cv::Point2f> cnrs_w;
        for (int i = 0; i < cnrs.size(); i++)
        {
            cv::Point3f wp = mVision.getWorldCoordinate(cnrs[i], image_depth);
            std::cout << "New corner point: " << wp << std::endl;
            cnrs_w.push_back(cv::Point2f(wp.x, wp.y));
        }
        // Update fabricCorner_C
        // (if translate, cnrs_w should be closed to fabricCorner_R)
        // (if rotate, assume it's a rigid motion, the rotated cnrs_w should be closed to fabricCorner_R)
        // (assume it's a deformative motion, the rotated cnrs_w should be closed to fabricCorner_C)
        // We assume all parts of fabric follows a rigid motion without deformation

        // Nearest point updating method can only be applied to the reference point, other points may have large position errors due to deformation.

        // find the nearest cnrs_w from fabricCorner_R
        int s_idx = 0;
        double smallest_dist = 999;
        for (int j = 0; j < cnrs_w.size(); j++)
        {
            double dist = cv::norm(state_cr.cnr[ref_id] - cnrs_w[j]);
            if (dist < smallest_dist)
            {
                smallest_dist = dist;
                s_idx = j;
            }
        }
        // s_idx in cnrs corresponds to ref_id in state_c.cnr
        int idx_c = s_idx;
        int idx_cr = ref_id;
        int size = state_c.cnr.size();
        for (int i = 0; i < state_cr.cnr.size(); i++)
        {
            state_c.cnr[idx_cr] = cnrs_w[idx_c];
            state_c.cnr_x[idx_cr] = cnrs[idx_c];
            idx_cr = (idx_cr + 1) % size;
            idx_c = (idx_c + 1) % size;
        }

        // Update corner direction after corner points are updated
        int mod = state_cr.cnr.size();
        for (int i = 0; i < state_cr.cnr.size(); i++)
        {
            int il = ((i - 1) % mod + mod) % mod;
            int ir = ((i + 1) % mod + mod) % mod;

            state_c.cnr_L[i] = getDirToEndB(state_c.cnr_x, i, il, grasp_length, image_depth);
            state_c.cnr_R[i] = getDirToEndB(state_c.cnr_x, i, ir, grasp_length, image_depth);
        }
    }

    void updateContour(std::vector<cv::Point> cnt, cv::Mat image_depth)
    {
        state_c.ctr.clear();
        state_c.ctr.resize(cnt.size());
        // init the initial coordinates of fabric contour
        double totHeight = 0;
        center = cv::Point2f(0, 0);
        for (int i = 0; i < cnt.size(); i++)
        {
            cv::Point3f wp = mVision.getWorldCoordinate(cnt[i], image_depth);
            state_c.ctr[i] = cv::Point2f(wp.x, wp.y);
            totHeight += wp.z;
            center += state_c.ctr[i];
        }
        state_c.ctr_x = cnt;

        int size = cnt.size();
        state_c.height = totHeight / size;
        state_c.center = center / size;
    }

    // state_c should be initialized beforehand
    void setTarget(cv::Point2f t, double rad)
    {
        state_c.copyTo(state_t);
        transform(state_t, t, state_c.center, rad);
    }

    void update()
    {
        // update the new position of main features, copy from fabricCorner_R to fabricCorner_C
        for (int i = 0; i < state_c.cnr.size(); i++)
        {
            state_c.cnr[i] = state_cr.cnr[i];
        }
    }

    // void translate(cv::Point2f t)
    // {
    //     fabricCorner_R.clear();
    //     fabricCorner_R.resize(fabricCorner_C.size());
    //     for (int i = 0; i < fabricCorner_C.size(); i++)
    //     {
    //         fabricCorner_R[i] = fabricCorner_C[i] + t;
    //     }
    // }

    cv::Point2f RotatePoint(const cv::Point2f &p, float rad)
    {
        const float x = std::cos(rad) * p.x - std::sin(rad) * p.y;
        const float y = std::sin(rad) * p.x + std::cos(rad) * p.y;

        const cv::Point2f rot_p(x, y);
        return rot_p;
    }

    cv::Point2f RotatePoint(const cv::Point2f &cen_pt, const cv::Point2f &p, float rad)
    {
        const cv::Point2f trans_pt = p - cen_pt;
        const cv::Point2f rot_pt = RotatePoint(trans_pt, rad);
        const cv::Point2f fin_pt = rot_pt + cen_pt;

        return fin_pt;
    }

    // void rotate2D(cv::Point3f center, double radian)
    // {
    //     fabricCorner_R.clear();
    //     fabricCorner_R.resize(fabricCorner_C.size());
    //     cv::Point2f c2d(center.x, center.y);
    //     for (int i = 0; i < fabricCorner_C.size(); i++)
    //     {
    //         cv::Point2f p2d(fabricCorner_C[i].x, fabricCorner_C[i].y);
    //         cv::Point2f p2d_r = RotatePoint(c2d, p2d, radian);
    //         fabricCorner_R[i].x = p2d_r.x;
    //         fabricCorner_R[i].y = p2d_r.y;
    //     }

    //     for (int i = 0; i < fabricCornerDirL_C.size(); i++)
    //     {
    //         fabricCornerDirL_R[i] = RotatePoint(fabricCornerDirL_C[i], radian);
    //         fabricCornerDirR_R[i] = RotatePoint(fabricCornerDirR_C[i], radian);
    //     }
    // }

    // bool graspToRequiredState()
    // {
    //     if (!dominatedGrasped)
    //     {
    //     }
    // }

    // int smooth(double allowance)
    // {
    //     for (int i = 0; i < fabricCorner_C.size(); i++)
    //     {
    //     }
    // }
};

#endif