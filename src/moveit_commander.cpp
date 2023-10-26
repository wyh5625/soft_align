#include "moveit_commander.h"
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "controller_manager_msgs/SwitchController.h"



static const string PLANNING_GROUP = "manipulator";
static const double cartesion_speed_scaling = 0.2;
static const double cartesion_acceleration_scaling = 0.2;
static const double joint_speed_scaling = 0.2;
static const double joint_acceleration_scaling = 0.2;
static const double PI = 3.141592653;

std::vector<string> pos_controllers = {"scaled_pos_traj_controller"};
std::vector<string> vel_controllers = {"joint_group_vel_controller"};

bool switchController(std::vector<string> star_controllers, std::vector<string> stop_controllers)
{
    controller_manager_msgs::SwitchControllerRequest req;
    controller_manager_msgs::SwitchControllerResponse res;
    // Choose which controller to load/unload
    req.start_controllers = star_controllers;
    req.stop_controllers = stop_controllers;
    req.strictness = 1;
    req.start_asap = true;
    req.timeout = 2.;
    ros::service::call("/controller_manager/switch_controller", req, res);
    return res.ok;
}

bool MoveitCommander::switchToPosController(){
    controller_manager_msgs::SwitchControllerRequest req;
    controller_manager_msgs::SwitchControllerResponse res;
    // Choose which controller to load/unload
    req.start_controllers = pos_controllers;
    req.stop_controllers = vel_controllers;
    req.strictness = 1;
    req.start_asap = true;
    req.timeout = 2.;
    ros::service::call("/controller_manager/switch_controller", req, res);
    return res.ok;
}

bool MoveitCommander::switchToVelController(){
    controller_manager_msgs::SwitchControllerRequest req;
    controller_manager_msgs::SwitchControllerResponse res;
    // Choose which controller to load/unload
    req.start_controllers = vel_controllers;
    req.stop_controllers = pos_controllers;
    req.strictness = 1;
    req.start_asap = true;
    req.timeout = 2.;
    ros::service::call("/controller_manager/switch_controller", req, res);
    return res.ok;
}

MoveitCommander::MoveitCommander()
{

    // constraints of collision with environment
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Add collision objects
    // Create a collision object that will be added to the scene
    moveit_msgs::CollisionObject collision_object, collision_object1, collision_object2, collision_object3;
    collision_object.header.frame_id = "world";  // Replace with the correct frame ID if needed
    collision_object1.header.frame_id = "world"; 
    collision_object2.header.frame_id = "world"; 
    collision_object3.header.frame_id = "world"; 

    // Set the ID of the object
    collision_object.id = "table";
    collision_object1.id = "wall";
    collision_object2.id = "stand1";
    collision_object3.id = "stand2";

    // Define the shape and dimensions of the object (table)
    shape_msgs::SolidPrimitive primitive, primitive1, primitive2, primitive3;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.8;

    primitive1.type = primitive.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.1;
    primitive1.dimensions[1] = 1.5;
    primitive1.dimensions[2] = 2.0;

    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.04;
    primitive2.dimensions[1] = 0.04;
    primitive2.dimensions[2] = 1.5;

    primitive3.type = primitive.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 0.04;
    primitive3.dimensions[1] = 0.04;
    primitive3.dimensions[2] = 1.5;

    // Define the pose of the object (table)
    geometry_msgs::Pose table_pose, wall_pose, stand_pose1, stand_pose2;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = -0.9;
    table_pose.position.y = 0.15;
    table_pose.position.z = -0.2;

    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = 0.45;
    wall_pose.position.y = 0.15;
    wall_pose.position.z = 0.4;

    stand_pose1.orientation.w = 1.0;
    stand_pose1.position.x = -0.32;
    stand_pose1.position.y = -0.56;
    stand_pose1.position.z = 0.95;

    stand_pose2.orientation.w = 1.0;
    stand_pose2.position.x = -0.32;
    stand_pose2.position.y = 0.86;
    stand_pose2.position.z = 0.95;


    // Add shape and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(wall_pose);
    collision_object1.operation = collision_object.ADD;

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(stand_pose1);
    collision_object2.operation = collision_object.ADD;

    collision_object3.primitives.push_back(primitive3);
    collision_object3.primitive_poses.push_back(stand_pose2);
    collision_object3.operation = collision_object.ADD;

    // Add the collision object into the world
    planning_scene_interface.applyCollisionObject(collision_object);
    planning_scene_interface.applyCollisionObject(collision_object1);
    planning_scene_interface.applyCollisionObject(collision_object2);
    planning_scene_interface.applyCollisionObject(collision_object3);


    // Constraints of robot joints

    // ros::init(NULL, NULL, "robot");
    moveGroup = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    // const moveit::core::JointModelGroup* joint_model_group = (*moveGroup).getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // this->joint_model_group = joint_model_group;
    visual_tools = new moveit_visual_tools::MoveItVisualTools("world");

    moveit_msgs::JointConstraint jcl,jcb,jcw1,jcw2;
    jcl.joint_name = "shoulder_lift_joint";
    // [-2.4, -0.55]
    jcl.position = -2.0;
    jcl.tolerance_above = 1.4;
    jcl.tolerance_below = 1.0;
    jcl.weight = 1.0;

    // jcb.joint_name = "shoulder_pan_joint";
    // // [-0.54, 1.62]
    // jcb.position = 0.54;
    // jcb.tolerance_above = 1.08;
    // jcb.tolerance_below = 1.08;
    // jcb.weight = 1.0;

    jcw1.joint_name = "wrist_1_joint";
    // -1.74
    jcw1.position = -1.74;
    jcw1.tolerance_above = 1;
    jcw1.tolerance_below = 1;
    jcw1.weight = 1.0;

    // jcw2.joint_name = "wrist_2_joint";
    // // [1, 2]
    // jcw2.position = 1.5;
    // jcw2.tolerance_above = 0.5;
    // jcw2.tolerance_below = 0.5;
    // jcw2.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.joint_constraints.push_back(jcl);
    // test_constraints.joint_constraints.push_back(jcb);
    // test_constraints.joint_constraints.push_back(jcw1);
    // test_constraints.joint_constraints.push_back(jcw2);
    moveGroup->setPathConstraints(test_constraints);

    moveGroup->setMaxVelocityScalingFactor(0.1);

    init();
}

MoveitCommander::~MoveitCommander()
{
    delete moveGroup;
    delete visual_tools;
}

void MoveitCommander::init()
{

    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
}

// std::vector<double> MoveitCommander::getCurrentJointPosition(){
//     std::vector<double> joint_group_positions;
//     moveit::core::RobotStatePtr current_state = moveGroup->getCurrentState();
//     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//     return joint_group_positions;
// }

// void MoveitCommander::plan_and_execute(geometry_msgs::Pose curr_pose, geometry_msgs::Pose target_pose){
//     moveGroup->setPoseTarget(target_pose);

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     bool success = (moveGroup->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success)
//     {
//         visual_tools->deleteAllMarkers();
//         visual_tools->publishTrajectoryLine(my_plan.trajectory_, moveGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
//         visual_tools->trigger();
//         // visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to take the move");
//         moveGroup->move();
//     }
//     else
//     {

//         std::cout << "Failed to plan the path to the new joint." << std::endl;
//         visual_tools->deleteAllMarkers();
//         // visual_tools->publishAxisLabeled(target_pose, "unreachable pose");
//         visual_tools->trigger();
//     }
// }

void MoveitCommander::plan_and_execute(std::vector<geometry_msgs::Pose> waypoints){
    moveit_msgs::RobotTrajectory trajectory_msg;
    bool executed_success = false;
    double fraction = 0;
    while(!executed_success){

        while(fraction < 0.8){
            fraction = moveGroup->computeCartesianPath(waypoints, 0.02, 0.0, trajectory_msg);
            std::cout << "The fraction of planned: " << fraction << std::endl;
        }
        

        robot_trajectory::RobotTrajectory rt(moveGroup->getCurrentState()->getRobotModel(), PLANNING_GROUP);
        rt.setRobotTrajectoryMsg(*(moveGroup->getCurrentState().get()), trajectory_msg);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // trajectory_processing::IterativeSplineParameterization isp;
        // trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool ItSuccess = iptp.computeTimeStamps(rt, cartesion_speed_scaling, cartesion_acceleration_scaling);  // computeTimeStamps(trajectory, max_vel, max_acce)

        ROS_INFO("Computed time stamp %s",ItSuccess?"SUCCEDED":"FAILED");

        rt.getRobotTrajectoryMsg(trajectory_msg);

        //std::cout << trajectory_msg << std::endl;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        //moveit::planning_interface::MoveGroup::Plan plan;
        plan.trajectory_ = trajectory_msg;

        visual_tools->deleteAllMarkers();
        visual_tools->publishTrajectoryLine(plan.trajectory_, moveGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        visual_tools->trigger();

        visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to take the move");

        //std::cout << "The execution of the planned trajectory failed, plan it again!";  
        if(moveGroup->execute(plan)){
            executed_success = true;
        }else{
            std::cout << "The execution of the planned trajectory failed, plan it again!";
        }   
    }

}

void MoveitCommander::calibrate_eelink()
{
    switchController(pos_controllers, vel_controllers);

    geometry_msgs::Pose target_pose, curr_pose;
    // target_pose.orientation.w = 1.0;
    curr_pose = getPose();

    std::string ref_frame = moveGroup->getPoseReferenceFrame();
    std::cout << "Move group reference frame: " << ref_frame << std::endl;

    // the default orientation of eef
    tf2::Quaternion q_eef;
    q_eef.setRPY(0, PI/2, PI);

    q_eef.normalize();
    tf2::convert(q_eef, target_pose.orientation);

    // target_pose.orientation = curr_pose.orientation;

    target_pose.position = curr_pose.position;

    //moveGroup->setPoseTarget(target_pose);

    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(curr_pose);
    // waypoints.push_back(target_pose);

    // moveit_msgs::RobotTrajectory rt;

    // moveGroup->setMaxVelocityScalingFactor(speed_scaling);

    std::vector<geometry_msgs::Pose> waypoints = {target_pose};
    plan_and_execute(waypoints);
    // double fraction = moveGroup->computeCartesianPath(waypoints, 0.01, 0.0, rt);

    // //moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // if (fraction > 0.8)
    // {
    //     visual_tools->deleteAllMarkers();
    //     visual_tools->publishTrajectoryLine(rt, moveGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
    //     visual_tools->trigger();
    //     visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to take the move");
    //     moveGroup->execute(rt);
    //     return true;
    // }
    // else
    // {

    //     std::cout << "Failed to plan the path to " << target_pose.position.x << "," << target_pose.position.y << "," << target_pose.position.z << std::endl;
    //     visual_tools->deleteAllMarkers();
    //     visual_tools->publishAxisLabeled(target_pose, "unreachable pose");
    //     visual_tools->trigger();
    //     return false;
    // }
}

void MoveitCommander::rviz_prompt(string msg)
{
    visual_tools->prompt(msg);
}

void MoveitCommander::goToPos(double x, double y, double z)
{
    switchController(pos_controllers, vel_controllers);

    geometry_msgs::Pose target_pose, curr_pose;
    // target_pose.orientation.w = 1.0;
    curr_pose = getPose();

    // the default orientation of eef
    // tf2::Quaternion q_eef;
    // q_eef.setRPY(0, 1.5708, 0);

    // q_eef.normalize();
    // tf2::convert(q_eef, target_pose.orientation);
    target_pose.orientation = curr_pose.orientation;

    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    //moveGroup->setMaxVelocityScalingFactor(speed_scaling);
    // moveGroup->setMaxVelocityScalingFactor(speed_scaling);
    // moveGroup->setMaxAccelerationScalingFactor(acceleration_scaling);

    std::vector<geometry_msgs::Pose> waypoints = {target_pose};
    plan_and_execute(waypoints);
}

// go to the position and rotate eef joint about rot
void MoveitCommander::goToBarPose(double pos[3], double rot)
{
    switchController(pos_controllers, vel_controllers);

    geometry_msgs::Pose target_pose, curr_pose;
    // target_pose.orientation.w = 1.0;

    curr_pose = getPose();
    // the default orientation of eef
    tf2::Quaternion q_eef, q_rot, q_curr;

    tf2::convert(curr_pose.orientation, q_curr);
    // q_eef.setRPY(0, 1.5708, 0);

    tf2::Vector3 axis(0, 0, 1);
    q_rot.setRotation(axis, rot);

    q_eef = q_rot * q_curr;
    q_eef.normalize();

    tf2::convert(q_eef, target_pose.orientation);

    target_pose.position.x = pos[0];
    target_pose.position.y = pos[1];
    target_pose.position.z = pos[2];

    // moveGroup->setMaxVelocityScalingFactor(speed_scaling);
    // moveGroup->setMaxAccelerationScalingFactor(acceleration_scaling);
    
    std::vector<geometry_msgs::Pose> waypoints = {target_pose};
    plan_and_execute(waypoints);
}

void MoveitCommander::goByCurve(std::vector<std::vector<double>> pos, std::vector<double> rots){
    switchController(pos_controllers, vel_controllers);

    geometry_msgs::Pose curr_pose;
    std::vector<geometry_msgs::Pose> waypoints;

    // target_pose.orientation.w = 1.0;

    curr_pose = getPose();
    // the default orientation of eef
    tf2::Quaternion q_eef, q_rot, q_curr;

    tf2::convert(curr_pose.orientation, q_curr);
    // q_eef.setRPY(0, 1.5708, 0);

    for(int i = 0; i < pos.size(); i++){
        geometry_msgs::Pose target_pose;

        tf2::Vector3 axis(0, 0, 1);
        q_rot.setRotation(axis, rots[i]);
        q_eef = q_rot * q_curr;
        q_eef.normalize();
        tf2::convert(q_eef, target_pose.orientation);

        target_pose.position.x = pos[i][0];
        target_pose.position.y = pos[i][1];
        target_pose.position.z = pos[i][2];

        waypoints.push_back(target_pose);
    }

    // moveGroup->setMaxVelocityScalingFactor(speed_scaling);
    // moveGroup->setMaxAccelerationScalingFactor(acceleration_scaling);

    plan_and_execute(waypoints);
}

bool MoveitCommander::goToJoint(std::vector<double> joints)
{
    std::vector<std::string> joint_names = moveGroup->getJoints();
    std::cout << "Print joint names in order:" << std::endl;
    for(int i = 0; i < joint_names.size(); i++){
        std::cout << joint_names[i] << std::endl;
    }

    std::vector<double> currentJoints = moveGroup->getCurrentJointValues();
    std::cout << "Print joint values in order:" << std::endl;
    for(int i = 0; i < currentJoints.size(); i++){
        std::cout << currentJoints[i] << std::endl;
    }
    switchController(pos_controllers, vel_controllers);

    //moveGroup->setMaxVelocityScalingFactor(speed_scaling);
    moveGroup->setJointValueTarget(joints);
    moveGroup->setMaxVelocityScalingFactor(joint_speed_scaling);
    moveGroup->setMaxAccelerationScalingFactor(joint_acceleration_scaling);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (moveGroup->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        visual_tools->deleteAllMarkers();
        visual_tools->publishTrajectoryLine(my_plan.trajectory_, moveGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
        visual_tools->trigger();
        visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to take the move");
        moveGroup->move();
        return true;
    }
    else
    {

        std::cout << "Failed to plan the path to the new joint." << std::endl;
        visual_tools->deleteAllMarkers();
        // visual_tools->publishAxisLabeled(target_pose, "unreachable pose");
        visual_tools->trigger();
        return false;
    }
}

void MoveitCommander::getPos(double pos[3])
{
    geometry_msgs::Pose curr_pose = moveGroup->getCurrentPose().pose;
    pos[0] = curr_pose.position.x;
    pos[1] = curr_pose.position.y;
    pos[2] = curr_pose.position.z;
}

geometry_msgs::Pose MoveitCommander::getPose()
{
    geometry_msgs::Pose curr_pose = moveGroup->getCurrentPose().pose;
    return curr_pose;
}

std::vector<double> MoveitCommander::getJointValues()
{
    return moveGroup->getCurrentJointValues();
}

void MoveitCommander::moveXYZ(double x, double y, double z)
{
    switchController(pos_controllers, vel_controllers);

    geometry_msgs::Pose target_pose, curr_pose;
    // target_pose.orientation.w = 1.0;

    curr_pose = getPose();

    target_pose.orientation = curr_pose.orientation;
    target_pose.position.x = curr_pose.position.x + x;
    target_pose.position.y = curr_pose.position.y + y;
    target_pose.position.z = curr_pose.position.z + z;

    //moveGroup->setMaxVelocityScalingFactor(speed_scaling);

    // std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(curr_pose);
    // waypoints.push_back(target_pose);
    std::vector<geometry_msgs::Pose> waypoints = {target_pose};
    plan_and_execute(waypoints);

}


string MoveitCommander::getPlanningFrame()
{
    return moveGroup->getPlanningFrame().c_str();
}

int MoveitCommander::main(int argc, char **argv)
{

    MoveitCommander mc;
    mc.init();

    ros::init(argc, argv, "soft_object_moveit_commander");

    ros::NodeHandle node;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Add collision objects
    // Create a collision object that will be added to the scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";  // Replace with the correct frame ID if needed

    // Set the ID of the object
    collision_object.id = "table";

    // Define the shape and dimensions of the object (table)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.8;

    // Define the pose of the object (table)
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = -0.9;
    table_pose.position.y = 0.15;
    table_pose.position.z = -0.2;

    // Add shape and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object into the world
    planning_scene_interface.applyCollisionObject(collision_object);


    const moveit::core::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;

    rviz_visual_tools::RvizVisualTools visual_tools("world");

    visual_tools.deleteAllMarkers();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    for (int i = 0; i < joint_group_positions.size(); i++)
    {
        std::cout << "Joint position " << i << ": " << joint_group_positions[i] << std::endl;
    }

    visual_tools.loadRemoteControl();

    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;
    target_pose1.position.z += 0.05;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    if (success)
    {
        move_group.move();
    }
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    return 0;
}