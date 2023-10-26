#include "Manipulator.h"
#include "helper_function.h"
#include <std_msgs/Float64MultiArray.h>

#define MINIMUM_ROTATE_DEGREE 5
#define HANG_HEIGHT 0.02
#define BAR_LENGTH 0.32
#define MIN_MOVEMENT 0.005
#define MAX_MOVEMENT 0.01

double foldDegree = 0.8;

Manipulator::Manipulator(MoveitCommander *robot)
{
    this->robot = robot;
}

void Manipulator::initPublisher(ros::Publisher *armArrowPub)
{
    this->armArrowPub = armArrowPub;
}

void Manipulator::ready()
{
    // some time it doesn't work, and is passed, so we can find it works like non_standby mode which is not support currently
    std::cout << "Go to ready position" << std::endl;
    // robot->goToPos(-0.5053848995476218, 0.21073543947752357, 0.7534501979117427);
    std::vector<double> joint_values = {0.1184341087937355, -1.1961529890643519, -1.7691872755633753, -1.7474177519427698, 1.5686171054840088, -1.4505766073810022};
    robot->goToJoint(joint_values);
    calibrate();
}

void Manipulator::calibrate()
{
    // robot->calibrate_eelink();
// [0.1184341087937355, -1.1961529890643519, -1.7691872755633753, -1.7474177519427698, 1.5686171054840088, -1.4505766073810022]
    double barMarkers[6] = {0.1184341087937355, -1.1961529890643519, -1.7691872755633753, -1.7474177519427698, 1.5686171054840088, -1.4505766073810022};
    double barVector[2];
    barVector[0] = barMarkers[0] - barMarkers[3];
    barVector[1] = barMarkers[1] - barMarkers[4] + 0.7;
    // two norms of the bar
    double bn0[2] = {-barVector[1], barVector[0]};
    gripperBarNormal[0] = bn0[0];
    gripperBarNormal[1] = bn0[1];
}

void Manipulator::grasp(double ori[2], double pos[3])
{

    double rot = getBarMinRotation(gripperBarNormal, ori);

    // move above the sticking position
    pos[2] += HANG_HEIGHT;
    robot->goToBarPose(pos, rot);

    gripperBarNormal[0] = ori[0];
    gripperBarNormal[1] = ori[1];

    // move down to stick the fabric
    robot->moveXYZ(0, 0, -HANG_HEIGHT);
    pos[2] -= HANG_HEIGHT;

    robot->rviz_prompt("Stick the bar tape on the cloth and press next in rviz window!");

    // Grasping Motion Control
    // A rviz trigger waiting for manual sticking

    // move up to hold the fabric
    // robot->moveXYZ(0, 0, HANG_HEIGHT);
    // pos[2] += HANG_HEIGHT;
}

void Manipulator::release(double ori[2], double pos[3])
{

    double rot = getRot2d(gripperBarNormal, ori);

    // move above the sticking position
    // pos[2] += HANG_HEIGHT;
    robot->goToBarPose(pos, rot);

    gripperBarNormal[0] = ori[0];
    gripperBarNormal[1] = ori[1];

    // move down to stick the fabric
    // robot->moveXYZ(0, 0, HANG_HEIGHT);
    // pos[2] += HANG_HEIGHT;

    robot->rviz_prompt("Release the bar tape on the cloth and press next in rviz window!");

    // Grasping Motion Control
    // A rviz trigger waiting for manual sticking

    // move up to hold the fabric
    // robot->moveXYZ(0, 0, HANG_HEIGHT);
    // pos[2] += HANG_HEIGHT;
}

void Manipulator::ungrasp()
{
}

void Manipulator::moveUp(double distance)
{
    robot->moveXYZ(0, 0, distance);
}

void Manipulator::rotateR(double center[2], double angle)
{
    int counts = 10;
    // get the eef current position
    double stickpos[3];
    robot->getPos(stickpos);

    std::cout << "sticked pos: " << stickpos[0] << " " << stickpos[1] << std::endl;

    std::vector<std::vector<double>> poss;
    std::vector<double> rots;
    for(int i = 0; i < counts; i++){
        double newPos[3];
        newPos[2] = stickpos[2];
        double rot = (i+1)*angle/counts;
        rotateEEF(center, rot, stickpos, newPos);

        std::vector<double> pos = {newPos[0], newPos[1], newPos[2]};
        poss.push_back(pos);
        rots.push_back(rot);
    }
    
    robot->goByCurve(poss, rots);
    //robot->goToBarPose(newPos, angle);

    double newOri[2];
    newOri[0] = gripperBarNormal[0] * cos(angle) - gripperBarNormal[1] * sin(angle);
    newOri[1] = gripperBarNormal[0] * sin(angle) + gripperBarNormal[1] * cos(angle);
    gripperBarNormal[0] = newOri[0];
    gripperBarNormal[1] = newOri[1];
}

void Manipulator::rotateTR(double center[2], double angle)
{
    // get the eef current position
    double stickpos[3];
    robot->getPos(stickpos);

    std::cout << "sticked pos: " << stickpos[0] << " " << stickpos[1] << std::endl;

    double newPos[3];
    newPos[2] = stickpos[2];

    rotateEEF(center, angle, stickpos, newPos);

    robot->goToBarPose(newPos, angle);

    double newOri[2];
    newOri[0] = gripperBarNormal[0] * cos(angle) - gripperBarNormal[1] * sin(angle);
    newOri[1] = gripperBarNormal[0] * sin(angle) + gripperBarNormal[1] * cos(angle);
    gripperBarNormal[0] = newOri[0];
    gripperBarNormal[1] = newOri[1];
}

void Manipulator::rotate(double ph1[2], double hole[2], double peg[2])
{
    // get the eef current position
    double stickpos[3];
    robot->getPos(stickpos);

    std::cout << "sticked pos: " << stickpos[0] << " " << stickpos[1] << std::endl;

    double rotate_radius = 0;

    double newStickingPos[3] = {0};
    // double angle;
    rotate_radius = rotateEEF(hole, peg, ph1, stickpos, newStickingPos);

    std::cout << "rotate angle: " << rotate_radius << std::endl;
    std::cout << "new sticked pos: " << newStickingPos[0] << " " << newStickingPos[1] << std::endl;

    if (abs(rotate_radius) > 0)
    {

        robot->goToBarPose(newStickingPos, rotate_radius);

        // no need to update sticking pos
        // update orientation of grasp and bar normal
        // singleGrasp.stickPos[0] = newStickingPos[0];
        // singleGrasp.stickPos[1] = newStickingPos[1];

        double newOri[2];
        newOri[0] = gripperBarNormal[0] * cos(rotate_radius) - gripperBarNormal[1] * sin(rotate_radius);
        newOri[1] = gripperBarNormal[0] * sin(rotate_radius) + gripperBarNormal[1] * cos(rotate_radius);
        gripperBarNormal[0] = newOri[0];
        gripperBarNormal[1] = newOri[1];

        // pihTasks.updateBarNorm(newOri);
    }
    else
    {
        std::cout << "No need to rotate!" << std::endl;
    }
}

void Manipulator::rotate(double ph1[2], double ph2[2], double hole[2], double peg[2])
{
    std::cout << "pegged hole 1: " << ph1[0] << " " << ph1[1] << std::endl;
    std::cout << "pegged hole 2: " << ph2[0] << " " << ph2[1] << std::endl;
    std::cout << "manipulated hole: " << hole[0] << " " << hole[1] << std::endl;
    std::cout << "peg: " << peg[0] << " " << peg[1] << std::endl;
    // get the eef current position
    double stickpos[3];
    robot->getPos(stickpos);

    std::cout << "sticked pos: " << stickpos[0] << " " << stickpos[1] << std::endl;

    double rotate_radius = 0;

    double newStickingPos[3] = {0};
    // double angle;
    if (ph1[0] == ph2[0] && ph1[1] == ph2[1])
    {
        std::cout << "the same point" << std::endl;
        // ph1 and ph2 are the same pegged hole, which forms a triangle controlled area
        rotate_radius = rotateEEF(hole, peg, ph1, stickpos, newStickingPos);
    }
    else
    {
        // Forms a quadrange controlled area
        // determine the rotation aixs(peg)
        double endA[2], endB[2];
        double normAB[2] = {gripperBarNormal[1], -gripperBarNormal[0]};

        endA[0] = stickpos[0] + 0.5 * BAR_LENGTH * normAB[0];
        endA[1] = stickpos[1] + 0.5 * BAR_LENGTH * normAB[1];
        endB[0] = stickpos[0] - 0.5 * BAR_LENGTH * normAB[0];
        endB[1] = stickpos[1] - 0.5 * BAR_LENGTH * normAB[1];

        bool inLeftTriangle = PointInTriangle(hole, endA, endB, ph1);
        bool inRightTriangle = PointInTriangle(hole, endA, endB, ph2);
        std::cout << "left | right " << inLeftTriangle << " " << inRightTriangle << std::endl;
        if (inLeftTriangle && inRightTriangle)
        {
            // no need to rotate in this area
        }
        else if (inLeftTriangle)
        {
            // rotate about left peg
            rotate_radius = rotateEEF(hole, peg, ph1, stickpos, newStickingPos);
        }
        else if (inRightTriangle)
        {
            // rotate about right
            rotate_radius = rotateEEF(hole, peg, ph2, stickpos, newStickingPos);
        }
        else
        {
            // no rotation
        }
    }

    std::cout << "rotate angle: " << rotate_radius << std::endl;
    std::cout << "new sticked pos: " << newStickingPos[0] << " " << newStickingPos[1] << std::endl;

    if (abs(rotate_radius) > 0)
    {

        robot->goToBarPose(newStickingPos, rotate_radius);

        // no need to update sticking pos
        // update orientation of grasp and bar normal
        // singleGrasp.stickPos[0] = newStickingPos[0];
        // singleGrasp.stickPos[1] = newStickingPos[1];

        double newOri[2];
        newOri[0] = gripperBarNormal[0] * cos(rotate_radius) - gripperBarNormal[1] * sin(rotate_radius);
        newOri[1] = gripperBarNormal[0] * sin(rotate_radius) + gripperBarNormal[1] * cos(rotate_radius);
        gripperBarNormal[0] = newOri[0];
        gripperBarNormal[1] = newOri[1];

        // pihTasks.updateBarNorm(newOri);
    }
    else
    {
        std::cout << "No need to rotate!" << std::endl;
    }
}

// fold length?
void Manipulator::fold(double peg_pos[3], double hole_pos[3], double stick_pos[3], double stick_ori[3], double max_fold_length)
{
    std::cout << "--- Fold part ---" << max_fold_length << std::endl;

    std::cout << "Max folding length: " << max_fold_length << std::endl;
    // singleGrasp.stickPos from robot position
    double peg_pj;

    double hole_v[2], peg_v[2];
    hole_v[0] = hole_pos[0] - stick_pos[0];
    hole_v[1] = hole_pos[1] - stick_pos[1];

    peg_v[0] = peg_pos[0] - stick_pos[0];
    peg_v[1] = peg_pos[1] - stick_pos[1];

    double hole_pj = hole_v[0] * stick_ori[0] + hole_v[1] * stick_ori[1];
    peg_pj = peg_v[0] * stick_ori[0] + peg_v[1] * stick_ori[1];

    std::cout << "hole_pj: " << hole_pj << std::endl;
    std::cout << "peg_pj: " << peg_pj << std::endl;

    // hole is between the corresponding peg and manipulator
    if (hole_pj < peg_pj + 0.01)
    {
        double foldLength_g;
        // max fold length from gripper
        // double maxFoldLength_g;
        // max fold length from hole

        // if (i == 0)
        // {
        //     // vectors from sticking point to left and right pegs
        //     double left_v[2], right_v[2];
        //     left_v[0] = rp_pos1[0] - stick_pos[0];
        //     left_v[1] = rp_pos1[1] - stick_pos[1];
        //     double left_pj = left_v[0] * stick_ori[0] + left_v[1] * stick_ori[1];

        //     right_v[0] = rp_pos2[0] - stick_pos[0];
        //     right_v[1] = rp_pos2[1] - stick_pos[1];
        //     double right_pj = right_v[0] * stick_ori[0] + right_v[1] * stick_ori[1];
        //     maxFoldLength_g = (left_pj < right_pj ? left_pj : right_pj);
        // }
        // else
        // {
        //     // last pegged hole as hinge
        //     double hinge_v[2];
        //     hinge_v[0] = singleGrasp.tasks[i - 1]->peg_pos[0] - stick_pos[0];
        //     hinge_v[1] = singleGrasp.tasks[i - 1]->peg_pos[1] - stick_pos[1];
        //     maxFoldLength_g = hinge_v[0] * stick_ori[0] + hinge_v[1] * stick_ori[1];
        // }
        foldLength_g = peg_pj + (max_fold_length - peg_pj) * foldDegree;
        std::cout << "actual fold length: " << foldLength_g << std::endl;

        // maxFoldLength_h = maxFoldLength_g - hole_pj;

        // Subtract some distance to eliminate visual error
        // foldLength -= 0.03;
        // rotation angle about the last peg so that the hole is on top of the corresponding peg
        // change virtual peg closer to last peg which will be used to make the hole to hung on top
        // new: virtual peg is the intersection of two arcs
        // calculate position of virtual peg according to cos formula
        double s = foldLength_g - hole_pj;
        double t = max_fold_length - peg_pj;
        double r = max_fold_length - foldLength_g;
        double cosTheta = (s * s + r * r - t * t) / (2 * s * r);
        double virPeg_pj = foldLength_g - s * (-cosTheta);
        double virPeg_h = s * sqrt(1 - cosTheta * cosTheta);

        // double virPeg_pj = peg_pj + 0.4* (maxFoldLength_g - peg_pj);

        double cosAngle = -cosTheta;
        double sinAngle = sqrt(1 - cosAngle * cosAngle);
        double planeMoveLength = (1 - cosAngle) * foldLength_g;
        double verticalMoveLength;
        if (planeMoveLength > foldLength_g)
        {
            planeMoveLength = foldLength_g - 0.04;
            verticalMoveLength = foldLength_g;
        }
        else
        {
            verticalMoveLength = sinAngle * foldLength_g;
        }

        double new_stick_pos[3];

        new_stick_pos[0] = stick_pos[0] + planeMoveLength * stick_ori[0];
        new_stick_pos[1] = stick_pos[1] + planeMoveLength * stick_ori[1];
        // substract (the raised height + some relax height)
        if (planeMoveLength < 0.05)
            new_stick_pos[2] = stick_pos[2] + (verticalMoveLength - 0.04);
        else
            new_stick_pos[2] = stick_pos[2] + (verticalMoveLength - 0.08);
        std::cout << "Fold the fabric about last peg!" << std::endl;

        std_msgs::Float64MultiArray ar;
        ar.data.resize(6);
        ar.data[0] = stick_pos[0];
        ar.data[1] = stick_pos[1];
        ar.data[2] = stick_pos[2];
        ar.data[3] = new_stick_pos[0];
        ar.data[4] = new_stick_pos[1];
        ar.data[5] = new_stick_pos[2];

        this->armArrowPub->publish(ar);

        robot->goToPos(new_stick_pos[0], new_stick_pos[1], new_stick_pos[2]);
    }
}

void Manipulator::unfold(double offset_target[3], double ori[3], double gripper_offset)
{
    // double currPos[3];
    // robot->getPos(currPos);
    // double stick_point_z = currPos[2] - gripper_offset;
    // double h = stick_point_z - hinge_peg_height;
    // double unfold_radius = sqrt(hinge_peg_distance*hinge_peg_distance + h*h);
    // double back_distance = unfold_radius - hinge_peg_distance;
    // robot->goToPos(currPos[0]-stick_ori[0]*back_distance,
    //                currPos[1]-stick_ori[1]*back_distance,
    //                currPos[2]-h);

    double currPos[3];
    robot->getPos(currPos);
    currPos[2] += gripper_offset;

    double eef2offsetTarget[3] = {offset_target[0] - currPos[0],
                                  offset_target[1] - currPos[1],
                                  offset_target[2] - currPos[2]};

    double distInNormDirect = eef2offsetTarget[0] * ori[0] + eef2offsetTarget[1] * ori[1];
    double radius_len = sqrt(distInNormDirect * distInNormDirect + eef2offsetTarget[2] * eef2offsetTarget[2]);

    double newStickPos[3] = {currPos[0] - (radius_len - distInNormDirect) * ori[0],
                             currPos[1] - (radius_len - distInNormDirect) * ori[1],
                             offset_target[2]};

    std_msgs::Float64MultiArray ar;
    ar.data.resize(6);
    ar.data[0] = currPos[0];
    ar.data[1] = currPos[1];
    ar.data[2] = currPos[2];
    ar.data[3] = newStickPos[0];
    ar.data[4] = newStickPos[1];
    ar.data[5] = newStickPos[2];

    this->armArrowPub->publish(ar);

    newStickPos[2] -= gripper_offset;
    robot->goToPos(newStickPos[0],
                   newStickPos[1],
                   newStickPos[2]);
}

void Manipulator::GPRControl()
{
}

// Moving on a 2D plane
void Manipulator::feedbackPeg(double distance[3], double delta)
{
    double currPos[3], newPos[3];
    robot->getPos(currPos);

    double movement[3] = {
        distance[0] * delta,
        distance[1] * delta,
        distance[2] * delta};

    double move_dist = sqrt(movement[0] * movement[0] + movement[1] * movement[1] + movement[2] * movement[2]);

    if (move_dist < MIN_MOVEMENT)
    {
        double ratio = MIN_MOVEMENT / move_dist;
        movement[0] *= ratio;
        movement[1] *= ratio;
        movement[2] *= ratio;
    }
    else if (move_dist > MAX_MOVEMENT)
    {
        double ratio = MAX_MOVEMENT / move_dist;
        movement[0] *= ratio;
        movement[1] *= ratio;
        movement[2] *= ratio;
    }

    // The direction of moving
    newPos[0] = currPos[0] + movement[0];
    newPos[1] = currPos[1] + movement[1];
    newPos[2] = currPos[2] + movement[2];

    double arrowLen = 0.05;
    double diffSLen = sqrt(movement[0] * movement[0] + movement[1] * movement[1] + movement[2] * movement[2]);
    double extendTarget[3] = {
        currPos[0] + movement[0] * arrowLen / diffSLen,
        currPos[1] + movement[1] * arrowLen / diffSLen,
        currPos[2] + movement[2] * arrowLen / diffSLen};

    std_msgs::Float64MultiArray ar_2_offset;

    ar_2_offset.data.resize(6);
    ar_2_offset.data[0] = currPos[0];
    ar_2_offset.data[1] = currPos[1];
    ar_2_offset.data[2] = currPos[2];

    ar_2_offset.data[3] = extendTarget[0];
    ar_2_offset.data[4] = extendTarget[1];
    ar_2_offset.data[5] = extendTarget[2];

    this->armArrowPub->publish(ar_2_offset);

    robot->goToPos(newPos[0], newPos[1], newPos[2]);
    // Moving distance
}

void Manipulator::translate(double distance[3])
{

    double currPos[3], newPos[3];
    robot->getPos(currPos);
    // The direction of moving
    newPos[0] = currPos[0] + distance[0];
    newPos[1] = currPos[1] + distance[1];
    newPos[2] = currPos[2] + distance[2];

    robot->goToPos(newPos[0], newPos[1], newPos[2]);
}