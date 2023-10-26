#include "moveit_commander.h"


class Manipulator{
    public:
        Manipulator(MoveitCommander* robot);
        void ready();
        void calibrate();

        // control primitives
        void grasp(double ori[2], double pos[3]);
        void release(double ori[2], double pos[3]);
        void ungrasp();
        void moveUp(double distance);
        void rotateR(double center[2], double angle);
        void rotateTR(double center[2], double angle);
        void rotate(double ph1[2], double hole[2], double peg[2]);
        void rotate(double ph1[2], double ph2[2], double hole[2], double peg[2]);
        void fold(double peg_pos[2], double hole_pos[2], double stick_pos[2], double stick_ori[2], double max_fold_length);
        void unfold(double offset_target[3], double ori[3], double gripper_offset);
        void GPRControl();
        void feedbackPeg(double distance[3], double delta);
        void translate(double distance[3]);

        // debugging tools
        void initPublisher(ros::Publisher *arrowPub);




    private:
        double liftingDistance;
        double fabricHeight;
        double gripperOffset;
        double gripperBarNormal[2];
        //robot
        MoveitCommander* robot;

        // publishers
        ros::Publisher* armArrowPub;


};