#ifndef FABRIC_STATE_H
#define FABRIC_STATE_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/types.hpp>
#include "new_type.h"


class FabricState
{
public:
    static int objectCount;
    int objectID;
    alignment_state aln_state;
    double height;
    cv::Point2f center;
    std::vector<bool> inView;
    std::vector<cv::Point2f> ctr;   // contour
    std::vector<cv::Point2f> cnr;   // corner
    std::vector<cv::Point2f> cnr_L; // left side of corner
    std::vector<cv::Point2f> cnr_R; // right side of corner

    // pixel(x)
    std::vector<cv::Point> ctr_x; // contour pixel
    std::vector<cv::Point> cnr_x; // corner pixel

    // contour idx of corner
    std::vector<int> cnr_idx_on_ctr;

    // orientation
    double ori;

    FabricState(){
        objectID = objectCount;
        objectCount ++;
    }

    void print(){
        // print points in cnr_x
        std::cout << "cnr_x: " << std::endl;
        for (int i = 0; i < cnr_x.size(); i++)
        {
            std::cout << cnr_x[i] << std::endl;
        }

        // print points in ctr
        std::cout << "cnr: " << std::endl;
        for (int i = 0; i < cnr.size(); i++)
        {
            std::cout << cnr[i] << std::endl;
        }

    }

    void copyTo(FabricState &fs)
    {
        fs.ctr.clear();
        fs.cnr.clear();
        fs.cnr_L.clear();
        fs.cnr_R.clear();
        fs.ctr_x.clear();
        fs.cnr_x.clear();

        fs.aln_state = aln_state;
        fs.height = height;
        fs.center = center;
        fs.ctr = ctr;
        fs.cnr = cnr;
        fs.cnr_L = cnr_L;
        fs.cnr_R = cnr_R;
        fs.ctr_x = ctr_x;
        fs.cnr_x = cnr_x;

        fs.inView = inView;
        fs.ori = ori;
    }

    

    cv::Point2f getNormalOfCornerDir(int id, std::string name)
    {
        cv::Point2f dl, dr;
        if (name == "L")
        {
            dl = cv::Point2f(cnr_L[id].y, -cnr_L[id].x);
            dr = cv::Point2f(-cnr_L[id].y, cnr_L[id].x);
            double sign = cnr_L[id].x * dl.y - cnr_L[id].y * dl.x;
            if(sign > 0)
                return dl;
            else
                return dr;
        }
        else if (name == "R")
        {
            dl = cv::Point2f(cnr_R[id].y, -cnr_R[id].x);
            dr = cv::Point2f(-cnr_R[id].y, cnr_R[id].x);
            double sign = cnr_R[id].x * dl.y - cnr_R[id].y * dl.x;
            if(sign < 0)
                return dl;
            else
                return dr;
        }

    }

    cv::Point3f getRigidForm()
    {
        cv::Point3f rigid_form;
        rigid_form.x = center.x;
        rigid_form.y = center.y;
        rigid_form.z = ori;
        return rigid_form;
    }

    // void updateCornerByMarker(std::vector<cv::Point> markers, std::vector<cv::Point> contour, FabricTransformer &ft){

    //     // if cnr_idx_on_ctr is empty, then initialize it
    //     // else, update the corner

    //     std::vector<int> cnr_idx_on_ctr_new(cnr_x.size(), -1);


    //     // Use the marker point to update the cloest corner point,use a vector to record the marker that is assigned to a corner, size of this vector is the same as corner, -1 of array means the corner is not assigned to any marker, otherwise, it has to compare the distance between corner and markers
    //     std::vector<int> marker_assigned(cnr_x.size(), -1);
    //     // A vector stores bool value, true means marker has been assigned to a corner
    //     std::vector<bool> marker_assigned_flag(markers.size(), false);

    //     for (int i = 0; i < markers.size(); i++)
    //     {
    //         double min_dist = 100000;
    //         int min_id = -1;
    //         for (int j = 0; j < cnr_x.size(); j++)
    //         {
                
    //             double dist = cv::norm(markers[i] - cnr_x[j]);
    //             if (dist < min_dist)
    //             {
    //                     min_dist = dist;
    //                     min_id = j;
    //             }
                
                
    //         }
    //         if (min_id != -1)
    //         {
    //             if (marker_assigned[min_id] == -1)
    //             {
    //                 marker_assigned[min_id] = i;
    //                 cnr_x[min_id] = markers[i];
    //                 marker_assigned_flag[i] = true;

    //                 // update the corner idx on contour
    //                 int closest_idx = getClosestIdx(markers[i], contour);
    //                 cnr_idx_on_ctr_new[min_id] = closest_idx;

    //             }else{
    //                 // compare the distance between the marker and the corner
    //                 double dist1 = cv::norm(markers[i] - cnr_x[min_id]);
    //                 double dist2 = cv::norm(markers[marker_assigned[min_id]] - cnr_x[min_id]);
    //                 if (dist1 < dist2)
    //                 {
    //                     marker_assigned_flag[marker_assigned[min_id]] = false;
    //                     marker_assigned[min_id] = i;
    //                     cnr_x[min_id] = markers[i];
    //                     marker_assigned_flag[i] = true;

    //                     // update the corner idx on contour
    //                     int closest_idx = getClosestIdx(markers[i], contour);
    //                     cnr_idx_on_ctr_new[min_id] = closest_idx;
    //                 }
    //             }
                
    //         }
    //     }

    //     std::vector<cv::Point> compared_markers;
    //     std::vector<int> compared_markers_idx;
    //     for (int i = 0; i < marker_assigned.size(); i++)
    //     {
    //         if (marker_assigned[i] != -1)
    //         {
    //             compared_markers.push_back(cnr_x[i]);
    //             compared_markers_idx.push_back(i);
    //         }
    //     }


    //     // update the corner that is not assigned to any marker
    //     for (int i = 0; i < marker_assigned_flag.size(); i++)
    //     {
    //         if (marker_assigned_flag[i] == false)
    //         {
    //             double minCost;
    //             int min_id = -1;
                
    //             for(int j = 0; j < cnr_x.size(); j++)
    //             {
    //                 if (marker_assigned[j] != -1)
    //                 {
    //                     std::vector<cv::Point> compared_markers_candidate = compared_markers;
    //                     std::vector<int> compared_markers_idx_candidate = compared_markers_idx;
    //                     // find the place in the compared_markers_idx_candidate that j is between
    //                     int idx = 0;
    //                     for (int k = 0; k < compared_markers_idx_candidate.size(); k++)
    //                     {
    //                         if (j > compared_markers_idx_candidate[k])
    //                         {
    //                             idx = k;
    //                         }
    //                     }
    //                     compared_markers_candidate.insert(compared_markers_candidate.begin() + idx, markers[i]);
    //                     compared_markers_idx_candidate.insert(compared_markers_idx_candidate.begin() + idx, j);

    //                     double cost = ft.matchCost(compared_markers_candidate, compared_markers_idx_candidate);
    //                     if (cost < minCost)
    //                     {
    //                         minCost = cost;
    //                         min_id = j;
    //                     }
    //                 }
    //             }
    //             if (min_id != -1)
    //             {
    //                 marker_assigned[min_id] = i;
    //                 cnr_x[min_id] = markers[i];
    //                 marker_assigned_flag[i] = true;

                
    //             }
                
    //         }
    //     }

    //     // update the corner that is not assigned to any marker, by relative position on the contour
    //     // for (int i = 0; i < cnr_x.size(); i++)
    //     // {
    //     //     if (marker_assigned[i] == -1)
    //     //     {
    //     //         // use a while loop to find the last index of corner that has been assigned to a marker
    //     //         // the last corner of first corner is the last corner of the corner vector
    //     //         int last_assigned_id = -1;
    //     //         int j = i;
    //     //         while (last_assigned_id == -1)
    //     //         {
    //     //             if (j == 0)
    //     //             {
    //     //                 j = cnr_x.size() - 1;
    //     //                 if (marker_assigned[j] != -1)
    //     //                 {
    //     //                     last_assigned_id = j;
    //     //                 }
    //     //             }
    //     //             else
    //     //             {
    //     //                 j--;
    //     //                 if (marker_assigned[j] != -1)
    //     //                 {
    //     //                     last_assigned_id = j;
    //     //                 }
    //     //             }
    //     //         }

    //     //         // use the same method to find index of the next corner
    //     //         int next_assigned_id = -1;
    //     //         j = i;
    //     //         while (next_assigned_id == -1)
    //     //         {
    //     //             if (j == cnr_x.size() - 1)
    //     //             {
    //     //                 j = 0;
    //     //                 if (marker_assigned[j] != -1)
    //     //                 {
    //     //                     next_assigned_id = j;
    //     //                 }
    //     //             }
    //     //             else
    //     //             {
    //     //                 j++;
    //     //                 if (marker_assigned[j] != -1)
    //     //                 {
    //     //                     next_assigned_id = j;
    //     //                 }
    //     //             }
    //     //         }
                
                
    //     //         // from cnr_idx_on_ctr, find (cnr_idx_on_ctr[i] - cnr_idx_on_ctr[last_assigned_id])/ (cnr_idx_on_ctr[next_assigned_id] - cnr_idx_on_ctr[last_assigned_id])
    //     //         // find number of points between last_assigned_id and next_assigned_id
    //     //         int num_points = 0;
    //     //         if (next_assigned_id > last_assigned_id)
    //     //         {
    //     //             num_points = next_assigned_id - last_assigned_id;
    //     //         }
    //     //         else
    //     //         {
    //     //             num_points = next_assigned_id + contour.size() - last_assigned_id;
    //     //         }

    //     //         // find number of points between last_assigned_id and i
    //     //         int num_points_i = 0;
    //     //         if (i > last_assigned_id)
    //     //         {
    //     //             num_points_i = i - last_assigned_id;
    //     //         }
    //     //         else
    //     //         {
    //     //             num_points_i = i + contour.size() - last_assigned_id;
    //     //         }

    //     //         // find the relative position of the invisible corner on the contour, with respect to last_assigned_id and next_assigned_id
    //     //         int relative_pos_old = num_points_i / num_points;
                
    //     //         // cnr_idx_on_ctr_new contains the new index of the corner on the contour
    //     //         // find number of points between last_assigned_id and next_assigned_id in the new contour
    //     //         int num_points_new = 0;
    //     //         if (cnr_idx_on_ctr_new[next_assigned_id] > cnr_idx_on_ctr_new[last_assigned_id])
    //     //         {
    //     //             num_points_new = cnr_idx_on_ctr_new[next_assigned_id] - cnr_idx_on_ctr_new[last_assigned_id];
    //     //         }
    //     //         else
    //     //         {
    //     //             num_points_new = cnr_idx_on_ctr_new[next_assigned_id] + contour.size() - cnr_idx_on_ctr_new[last_assigned_id];
    //     //         }

    //     //         // find the index of the invisible corner on the contour
    //     //         int idx_on_ctr = (cnr_idx_on_ctr_new[last_assigned_id] + relative_pos_old * num_points_new)%contour.size();

    //     //         cnr_idx_on_ctr_new[i] = idx_on_ctr;

                
    //     //     }
    //     // }

    //     // cnr_idx_on_ctr = cnr_idx_on_ctr_new;

    //     // // update the corner position on the contour
    //     // for (int i = 0; i < cnr_x.size(); i++)
    //     // {
    //     //     cnr_x[i] = contour[cnr_idx_on_ctr[i]];
    //     // }

    //     // find the relative position of the invisible corner on the contour
    //     // for (int i = 0; i < cnr_x.size(); i++)
    //     // {
    //     //     if (marker_assigned[i] == -1)
    //     //     {
    //     //         // find the closest point on the contour
    //     //         double min_dist = 100000;
    //     //         int min_id = -1;
    //     //         for (int j = 0; j < contour.size(); j++)
    //     //         {
    //     //             double dist = cv::norm(contour[j] - cnr_x[i]);
    //     //             if (dist < min_dist)
    //     //             {
    //     //                 min_dist = dist;
    //     //                 min_id = j;
    //     //             }
    //     //         }
    //     //         cnr_x[i] = contour[min_id];
    //     //     }
    //     // }


    //     // for (int i = 0; i < marker_assigned_flag.size(); i++)
    //     // {
    //     //     if (marker_assigned_flag[i] == false)
    //     //     {
    //     //         // user input a int number to select the corner
    //     //         int corner_id = -1;
    //     //         std::cout << "Please input the corner id that you want to assign to marker at " << markers[i] << std::endl;
    //     //         std::cin >> corner_id;
    //     //         if (corner_id >= 0 && corner_id < cnr_x.size())
    //     //         {
    //     //             cnr_x[corner_id] = markers[i];
    //     //             marker_assigned[corner_id] = i;
    //     //             marker_assigned_flag[i] = true;
    //     //         }
    //     //     }


    //     // }

    //     // for marker that has not been assigned to a corner

            

    //     // std::vector<int> marker_assigned_contour(cnr_x.size(), -1);
    //     // // for each marker that has not been assigned to a corner, find the index of closest point on contour, and save in marker_assigned_contour
    //     // for (int i = 0; i < marker_assigned_flag.size(); i++)
    //     // {
    //     //     if (marker_assigned_flag[i] == false)
    //     //     {
    //     //         double min_dist = 100000;
    //     //         int min_id = -1;
    //     //         for (int j = 0; j < contour.size(); j++)
    //     //         {
    //     //             double dist = cv::norm(cnr_x[i] - contour[j]);
    //     //             if (dist < min_dist)
    //     //             {
    //     //                 min_dist = dist;
    //     //                 min_id = j;
    //     //             }
    //     //         }
    //     //         if (min_id != -1)
    //     //         {
    //     //             marker_assigned_contour[i] = min_id;
    //     //         }
    //     //     }
    //     // }

    //     // // for each corner, if it has been assigned to a marker, find the index of closest point on contour, and save in corner_assigned_contour
    //     // // else set it to -1
    //     // std::vector<int> corner_assigned_contour(cnr_x.size(), -1);
    //     // for (int i = 0; i < cnr_x.size(); i++)
    //     // {
    //     //     if (marker_assigned[i] != -1)
    //     //     {
    //     //         double min_dist = 100000;
    //     //         int min_id = -1;
    //     //         for (int j = 0; j < contour.size(); j++)
    //     //         {
    //     //             double dist = cv::norm(cnr_x[i] - contour[j]);
    //     //             if (dist < min_dist)
    //     //             {
    //     //                 min_dist = dist;
    //     //                 min_id = j;
    //     //             }
    //     //         }
    //     //         if (min_id != -1)
    //     //         {
    //     //             corner_assigned_contour[i] = min_id;
    //     //         }
    //     //     }
    //     // }


    //     // /////////////

    //     // // for each marker_assigned_flag, if it's false, find the index of cloest contour point, 
    //     // // and for loop to compare every element in corner_assigned_contour which is not -1 with this index
    //     // // if this index is just greater then the element, then set int variable target_corner_id to the index of this element
    //     // for (int i = 0; i < marker_assigned_flag.size(); i++)
    //     // {
    //     //     if (marker_assigned_flag[i] == false)
    //     //     {
    //     //         // find the index of cloest contour point
    //     //         double min_dist = 100000;
    //     //         int min_id = -1;
    //     //         for (int j = 0; j < contour.size(); j++)
    //     //         {
    //     //             double dist = cv::norm(markers[i] - contour[j]);
    //     //             if (dist < min_dist)
    //     //             {
    //     //                 min_dist = dist;
    //     //                 min_id = j;
    //     //             }
    //     //         }
    //     //         if (min_id != -1)
    //     //         {
    //     //             // for loop to compare every element in corner_assigned_contour which is not -1 with this index
    //     //             int target_corner_id = -1;
    //     //             for (int k = 0; k < corner_assigned_contour.size(); k++)
    //     //             {
    //     //                 if (corner_assigned_contour[k] != -1)
    //     //                 {
    //     //                     if (corner_assigned_contour[k] > min_id)
    //     //                     {
    //     //                         if (target_corner_id == -1)
    //     //                         {
    //     //                             target_corner_id = k;
    //     //                         }else{
    //     //                             if (corner_assigned_contour[k] < corner_assigned_contour[target_corner_id])
    //     //                             {
    //     //                                 target_corner_id = k;
    //     //                             }
    //     //                         }
    //     //                     }
    //     //                 }
    //     //             }
    //     //             if (target_corner_id != -1)
    //     //             {
    //     //                 marker_assigned[target_corner_id] = i;
    //     //                 cnr_x[target_corner_id] = markers[i];
    //     //                 marker_assigned_flag[i] = true;
    //     //             }
    //     //         }
    //     //     }
    //     // }

        


        
        
        
        
        
        
        
        
        
        
        
    //     // for (int i = 0; i < marker_assigned_flag.size(); i++)
    //     // {
    //     //     if (marker_assigned_flag[i] == false)
    //     //     {
    //     //         double min_dist = 100000;
    //     //         int min_id = -1;
    //     //         for (int j = 0; j < cnr_x.size(); j++)
    //     //         {
    //     //             if (marker_assigned[j] == -1)
    //     //             {
    //     //                 double dist = cv::norm(markers[i] - cnr_x[j]);
    //     //                 if (dist < min_dist)
    //     //                 {
    //     //                     min_dist = dist;
    //     //                     min_id = j;
    //     //                 }
    //     //             }
    //     //         }
    //     //         if (min_id != -1)
    //     //         {
    //     //             marker_assigned[min_id] = i;
    //     //             cnr_x[min_id] = markers[i];
    //     //         }
    //     //     }
    //     // }


    // }
};

int FabricState::objectCount = 0;

#endif