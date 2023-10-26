#ifndef MOVEIT_COMMANDER_H_
#define MOVEIT_COMMANDER_H_

#include <string>
#include <ros/ros.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/RobotTrajectory.h>

using namespace std;



class MoveitCommander{
    public:
        MoveitCommander();
        ~MoveitCommander();
        void init();
        //std::vector<double> getCurrentJointPosition();
        void plan_and_execute(std::vector<geometry_msgs::Pose> waypoints);
        void calibrate_eelink();
        void rviz_prompt(string msg);
        void goToPos(double x, double y, double z);
        void goToBarPose(double pos[3], double rot);
        void goByCurve(std::vector<std::vector<double>> pos, std::vector<double> rots);
        bool goToJoint(std::vector<double> joints);
        void moveXYZ(double x, double y, double z);
        void getPos(double pos[3]);
        geometry_msgs::Pose getPose();
        string getPlanningFrame();
        std::vector<double> getJointValues();
        int main(int argc, char **argv);

        bool switchToPosController();
        bool switchToVelController();

    private:
        moveit::planning_interface::MoveGroupInterface *moveGroup;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::core::JointModelGroup* joint_model_group;
        moveit_visual_tools::MoveItVisualTools *visual_tools;
        

};

bool switchController(std::vector<string> star_controllers, std::vector<string> stop_controllers);


#endif