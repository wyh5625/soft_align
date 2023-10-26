#define MINIMUM_ROTATE_DEGREE 5

void HP(double q1[4], double q2[4], double qout[4]);
void getQuaternion(double v[3], double angle, double qout[4]);
double getRot2d(double v1[2], double v2[2]);
void rotateEEF(double center[2], double rot_diff, double currPos[2], double newPos[2]);
double rotateEEF(double holePos[2], double pegPos[2], double restrictPegPos[2], double eefPos[3], double neweefPos[3]);
double getBarMinRotation(double from[2], double to[2]);
// double getMinRotation(double from[2], double to[2]);
double sign(double p1[], double p2[], double p3[]);
bool PointInTriangle(double pt[], double v1[], double v2[], double v3[]);