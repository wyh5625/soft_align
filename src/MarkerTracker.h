#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "generateLabelings.h"
#include "FabricTransformer.h"
#include "FabricState.h"

namespace
{

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
        // std::cout << "smallest_dist: " << smallest_dist << std::endl;

        if (smallest_dist > 15)
        {
            return -1;
        }

        return s_idx;
    }

    int getIndex(std::vector<int> v, int K)
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
            if(order[i] == -1) {
                continue;
            }
            int idx = getIndex(indices, order[i]);
            out.push_back(markers[idx]);
        }

        return out;
    }
}

class Marker
{
    // Define properties and methods for Marker
};

// class Corner {
//     // Define properties and methods for Corner
//     bool isCertain;
//     std::vector<Marker> possibleMarkers;
// };

class Hypothesis
{
public:
    // Define properties and methods for Hypothesis
    std::vector<int> marker_idx;
    std::vector<int> cnr_idx;
};

class MarkerTracker
{
    
    // std::vector<Marker> markers;
    std::vector<bool> certain;
    std::vector<int> lostCount;
    int num_of_label;
    int uncertain_thres;
    FabricTransformer* ft;

public:
    std::vector<cv::Point> corners;
    std::vector<cv::Point> last_certain_markers;

    // fabric_transformer should already be initialized
    MarkerTracker(int num_of_label, int uncertain_thres, FabricTransformer* fabric_transformer)
    {
        this->num_of_label = num_of_label;
        this->uncertain_thres = uncertain_thres;
        this->ft = fabric_transformer;
    }

    void init(std::vector<cv::Point> markers, std::vector<cv::Point> contour, cv::Mat& imageShowDepth)
    {
        // init lostCount to 0
        lostCount = std::vector<int>(num_of_label, 0);

        // print num_of_label
        std::cout << "num_of_label: " << num_of_label << std::endl;

        std::cout << "certain size: " << certain.size() << std::endl;

        // init certain to true
        certain = std::vector<bool>(num_of_label, true);

        // pirnt size of certain
        std::cout << "certain size: " << certain.size() << std::endl;

        // mapping all markers to corners by template mapping
        FabricState fs = ft->getFabricStateFromData(markers, contour, imageShowDepth, 0.18);

        corners = fs.cnr_x;

        last_certain_markers = corners;
    }

    void updateCertainty(std::vector<Hypothesis> hypotheses)
    {
        // std::cout << "--------------------- 0 --------------" << std::endl;
        // Implementation
        for (int i = 0; i < corners.size(); i++)
        {
            if (lostCount[i] > uncertain_thres)
            {
                std::cout << "corner " << i << " changed to uncertain" << std::endl;
                certain[i] = false;
            }
            else
            {
                // std::cout << "corner " << i << " changed to certain" << std::endl;
                certain[i] = true;
            }
        }
        // std::cout << "--------------------- 1 --------------" << std::endl;
        // print size of hypotheses
        std::cout << "hypotheses size: " << hypotheses.size() << std::endl;
        for(int i = 0; i < hypotheses[0].marker_idx.size(); i++) {
            // check if label for each marker in each hypothesis is the same
            bool certain_marker = true;
            for(int j = 0; j < hypotheses.size(); j++) {
                // std::cout << "--------------------- 1.1 --------------" << std::endl;
                if(hypotheses[j].cnr_idx[i] != hypotheses[0].cnr_idx[i]) {
                    // std::cout << "--------------------- 1.2 --------------" << std::endl;
                    // if not, then the corner is uncertain
                    certain_marker = false;
                }
            }
            // std::cout << "--------------------- 2 --------------" << std::endl;
            // if all labels for the marker are the same, then the corner is certain
            if(certain_marker){
                std::cout << "corner " << hypotheses[0].cnr_idx[i] << " is certain" << std::endl;
                certain[hypotheses[0].cnr_idx[i]] = true;
                lostCount[hypotheses[0].cnr_idx[i]] = 0;
            }
        }


    }

    // markers should be in circlular order
    std::vector<std::pair<int, int>> assignMarkers(std::vector<cv::Point> markers, std::vector<cv::Point> contour)
    {
        std::vector<std::pair<int, int>> certain_labelings;

        std::vector<std::vector<int>> assigned_marker = std::vector<std::vector<int>>(corners.size(), std::vector<int>());

        // print size of contour
        std::cout << "contour size: " << contour.size() << std::endl;
        // print all points in markers
        for(int i = 0; i < markers.size(); i++) {
            std::cout << "marker " << i << ": " << markers[i] << std::endl;
        }
        // print all points in corners
        for(int i = 0; i < corners.size(); i++) {
            std::cout << "corner " << i << ": " << corners[i] << std::endl;
        }
        // std::vector<int> assigned_marker = std::vector<int>(markers.size(), -1);
        // std::vector<std::pair<int, int>> fixed_labels;

        for (int i = 0; i < markers.size(); i++)
        {
            // std::cout << "--------------------- a1 --------------" << std::endl;
            double min_dist = 100000;
            int min_id = -1;
            for (int j = 0; j < corners.size(); j++)
            {
                double dist = cv::norm(markers[i] - corners[j]);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_id = j;
                }
            }
            // std::cout << "--------------------- a2 --------------" << std::endl;
            // print min_id
            // std::cout << "min_id: " << min_id << std::endl;
            // // print size of certain
            // std::cout << "certain size: " << certain.size() << std::endl;
            // std::cout << "certain[min_id]: " << certain[min_id] << std::endl;
            // std::cout << "lostCount[min_id]: " << lostCount[min_id] << std::endl;

            // if there are multi markers assigned to the same corner, this corner should be made to uncertain
            if (certain[min_id])
            {
                assigned_marker[min_id].push_back(i);
            }

            // （test1）even if the corner is uncertain, we still assign the closest marker to it
            // assigned_marker[min_id].push_back(i);
            

            // std::cout << "--------------------- a3 --------------" << std::endl;
            // currently, we don't consider the case where more markers may be assigned to the same corner
            // assume all markers are distant to each other
        }

        // （test2）try to comment out this part next time for testing

        // （test3）this may be the solution， the distance of consecutive certain should be very close
        //  so record the pixel of each corner when it's certain, and when it becomes certain again (only when it's assigned a single marker), 
        //  check its distance to the previous certain corner, if it's too far, then it's not certain
        //  Certain is only for checking cover but not move.
        for (int i = 0; i < corners.size(); i++)
        {
            if (assigned_marker[i].size() == 1)
            {
                double dist = cv::norm(last_certain_markers[i] - markers[assigned_marker[i][0]]);
                if (dist > 20)
                {
                    certain[i] = false;
                }else{
                    // update corner
                    // corners[min_id] = markers[i];
                    // assigned_marker[i] = min_id;
                    lostCount[i] = 0;
                    std::pair<int, int> labeling = std::make_pair(assigned_marker[i][0], i);
                    certain_labelings.push_back(labeling);

                    last_certain_markers[i] = markers[assigned_marker[i][0]];
                }
            }else{
                certain[i] = false;
            }

        }

        return certain_labelings;
    }

    std::vector<Hypothesis> generateHypotheses(int num_of_labels, std::vector<std::pair<int, int>> fixed_labelings, int num_of_markers)
    {
        // init vector labels with index of corners
        std::vector<int> labels;
        for (int i = 0; i < num_of_labels; i++)
        {
            labels.push_back(i);
        }
        // init vector markers with index of markers
        std::vector<int> markers_idx;
        for (int i = 0; i < num_of_markers; i++)
        {
            markers_idx.push_back(i);
        }

        // Implementation
        auto results = generateLabelings(labels, fixed_labelings, markers_idx);

        std::vector<Hypothesis> hypotheses;
        for (const auto &result : results)
        {
            Hypothesis h;
            std::vector<int> idx;
            for (const auto &pair : result)
            {
                h.marker_idx.push_back(pair.first);
                h.cnr_idx.push_back(pair.second);
            }
            hypotheses.push_back(h);
        }
        return hypotheses;
    }

    double evaluateHypotheses(Hypothesis h, std::vector<cv::Point> circulated_marker, cv::Mat &imageShowDepth)
    {
        std::vector<cv::Point> markers;
        for (int i = 0; i < h.marker_idx.size(); i++)
        {
            markers.push_back(circulated_marker[h.marker_idx[i]]);
        }
        // Implementation
        double cost = ft->matchCost(markers, h.cnr_idx, imageShowDepth);
        return cost;
    }

    void updateCorners(Hypothesis h, std::vector<cv::Point> circulated_marker)
    {
        // Implementation
        // Update corner position
        // clear old corners points
        for (int i = 0; i < corners.size(); i++)
        {
            corners[i] = cv::Point(0, 0);
        }
        
        for (int i = 0; i < h.marker_idx.size(); i++)
        {
            corners[h.cnr_idx[i]] = circulated_marker[h.marker_idx[i]];
        }

        // for corner index that is not in h.cnr_idx, update its lost count
        for (int i = 0; i < corners.size(); i++)
        {
            if (std::find(h.cnr_idx.begin(), h.cnr_idx.end(), i) == h.cnr_idx.end())
            {
                lostCount[i]++;
            }
        }

    }

    Hypothesis selectHypotheses(std::vector<Hypothesis> hyps, std::vector<cv::Point> convexCnt, cv::Mat &imageShowDepth)
    {
        // Implementation
        double minCost = 100000;
        Hypothesis best_hyp;
        for (const auto &hyp : hyps)
        {
            double cost = evaluateHypotheses(hyp, convexCnt, imageShowDepth);
            if (cost < minCost)
            {
                minCost = cost;
                best_hyp = hyp;
            }
        }
        return best_hyp;
    }

    void update(std::vector<cv::Point> markers, std::vector<cv::Point> contour, cv::Mat &imageShowDepth)
    {
        // std::cout << "--------------------- a --------------" << std::endl;
        std::vector<std::pair<int, int>> fixed_labels;
        // order markers by contour
        std::vector<cv::Point> convexCnt = orderByContour(markers, contour);
        // std::cout << "--------------------- b --------------" << std::endl;
        std::vector<std::pair<int, int>> fixed_labelings = assignMarkers(convexCnt, contour);
        // std::cout << "--------------------- c --------------" << std::endl;
        std::vector<Hypothesis> all_hyps = generateHypotheses(num_of_label, fixed_labelings, convexCnt.size());
        // std::cout << "--------------------- d --------------" << std::endl;
        if(all_hyps.size() > 0)
        {
            Hypothesis best_h = selectHypotheses(all_hyps, convexCnt, imageShowDepth);
            // std::cout << "--------------------- e --------------" << std::endl;
            updateCertainty(all_hyps);

            updateCorners(best_h, convexCnt);

            if (convexCnt.size() == num_of_label){
                // set all corners to certain
                for(int i = 0; i < certain.size(); i++) {
                    certain[i] = true;
                }
                last_certain_markers = corners;
            }
            
            // std::cout << "--------------------- f --------------" << std::endl;
            
        }
        
    }
};
