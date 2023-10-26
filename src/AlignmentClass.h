#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/types.hpp>

class Grasp
{
public:
    cv::Point3f sticking_point;
    cv::Point3f orientation;
};

class Action
{
public:
    int type;
    cv::Point3f value;
    double angle;
};
class AbstractGrasp
{
public:
    int featureType; // The type of feature to grasp. eg. 0 for corner point, 1 for corner side.
    int featureID;   // The index of feature to grasp. eg. 0: LS of corner 0, 1: RS of corner 0, 2: LS of corner 1. etc.
};
class AbstractAction
{
public:
    int actionType;  // 0 for rotation, 1 for translation, 2 for pick and place, 3 for translation and rotation
    int referenceID; // eg. corner id
};

class AbstractManipulation
{
public:
    AbstractGrasp abGrasp;
    AbstractAction abAction;
};

class GraspAndMovement
{
public:
    // 0 for rotation, 1 for translation, 2 for pick n place
    int type;
    Grasp pick;
    Grasp place;
    cv::Point3f movement;
    cv::Point2f center;
    double angle;
    int ref_id;
};

class CornerAlign
{
public:
    int cnr_idx;
    std::string side;
    cv::Point2f point_from;
    cv::Point2f point_to;
    cv::Point2f dir_from;
    cv::Point2f dir_to;
    cv::Point2f pick_normal;
    cv::Point2f place_normal;
    double move_cos; // cos value of angle between pick line normal and moving dir
};

class Manipulation
{
public:
    Grasp mGrasp;
    std::vector<int> manipulatedKP;
    std::vector<Action> actions;
};