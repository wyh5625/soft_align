#include "helper_function.h"
#include <cmath>
#include <iostream>


void HP(double q1[4], double q2[4], double qout[4])
{
    qout[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    qout[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    qout[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    qout[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

void rotateQ(double v[3], double rq[4], double vout[3])
{
    // P' = RPR'
    // where R:rq, R':rq_, P:vq, P':rightP
    double rq_[4], vq[4];
    rq_[0] = rq[0];
    rq_[1] = -rq[1];
    rq_[2] = -rq[2];
    rq_[3] = -rq[3];

    vq[0] = 0;
    vq[1] = v[0];
    vq[2] = v[1];
    vq[3] = v[2];

    // leftP: RP, rightP:RPR'
    double leftP[4], rightP[4];
    HP(rq, vq, leftP);
    HP(leftP, rq_, rightP);

    // rightP[0] should always be 0
    vout[0] = rightP[1];
    vout[1] = rightP[2];
    vout[2] = rightP[3];
}

void getQuaternion(double v[3], double angle, double qout[4])
{
    // normalize v to unit vector
    double v_norm[3];
    double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v_norm[0] = v[0] / norm;
    v_norm[1] = v[1] / norm;
    v_norm[2] = v[2] / norm;

    qout[0] = cos(angle / 2);
    qout[1] = sin(angle / 2) * v_norm[0];
    qout[2] = sin(angle / 2) * v_norm[1];
    qout[3] = sin(angle / 2) * v_norm[2];
}

// calculate rotation from v1 to v2 in radians
double getRot2d(double v1[2], double v2[2])
{
    double norm1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
    double norm2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1]);
    double c = (v1[0] * v2[0] + v1[1] * v2[1]) / (norm1 * norm2);
    double z = v1[0] * v2[1] - v2[0] * v1[1];
    if (z >= 0)
    {
        return acos(c);
    }
    else
    {
        return -acos(c);
    }
}

void rotateEEF(double center[2], double rot_diff, double currPos[2], double newPos[2]){
    double axis[3] = {0, 0, 1};
    double quaternionBar[4];

    double h_v[2], p_v[2];
    h_v[0] = currPos[0] - center[0];
    h_v[1] = currPos[1] - center[1];

    getQuaternion(axis, rot_diff, quaternionBar);

    rotateQ(h_v, quaternionBar, p_v);

    newPos[0] = center[0] + p_v[0];
    newPos[1] = center[1] + p_v[1];
}

double rotateEEF(double holePos[2], double pegPos[2], double restrictPegPos[2], double eefPos[3], double neweefPos[3])
{
    double rot_diff;
    double h_v[2], p_v[2];
    // from the restricted peg to the manipulted hole
    h_v[0] = holePos[0] - restrictPegPos[0];
    h_v[1] = holePos[1] - restrictPegPos[1];

    // from the restricted peg to the corresponding peg
    p_v[0] = pegPos[0] - restrictPegPos[0];
    p_v[1] = pegPos[1] - restrictPegPos[1];

    rot_diff = getRot2d(h_v, p_v);
    std::cout << "Computed rotate radius: " << rot_diff << std::endl;
    std::cout << "Computed rotate radius abs: " << fabs(rot_diff) << std::endl;
    //if (fabs(rot_diff) > MINIMUM_ROTATE_DEGREE * 3.14 / 180)
    //{

        // vector from pegged hole to sticking point
        double sp_v0[3], sp_v1[3];
        sp_v0[0] = eefPos[0] - restrictPegPos[0];
        sp_v0[1] = eefPos[1] - restrictPegPos[1];
        sp_v0[2] = 0;

        double axis[3] = {0, 0, 1};
        double quaternionBar[4];

        getQuaternion(axis, rot_diff, quaternionBar);
        // final vector of sticking point from restricted peg
        rotateQ(sp_v0, quaternionBar, sp_v1);

        neweefPos[0] = restrictPegPos[0] + sp_v1[0];
        neweefPos[1] = restrictPegPos[1] + sp_v1[1];
        neweefPos[2] = eefPos[2];
        return rot_diff;
    // }
    // else
    // {
    //     return 0;
    // }
}

double getBarMinRotation(double from[2], double to[2])
{

    // find the minimum rotation for the bar norm to follow ob
    double rot_bn;
    double rot_bn0 = getRot2d(from, to);

    if (rot_bn0 > 3.141592 / 2)
    {
        rot_bn = rot_bn0 - 3.1415926;
    }
    else if (rot_bn0 < -3.141592 / 2)
    {
        rot_bn = rot_bn0 + 3.1415926;
    }
    else
    {
        rot_bn = rot_bn0;
    }
    return rot_bn;
}

// double getMinRotation(double from[2], double to[2])
// {

//     // find the minimum rotation for the bar norm to follow ob
//     double rot_bn;
//     double rot_bn0 = getRot2d(from, to);

//     // if (rot_bn0 > 3.141592 / 2)
//     // {
//     //     rot_bn = rot_bn0 - 3.1415926;
//     // }
//     // else if (rot_bn0 < -3.141592 / 2)
//     // {
//     //     rot_bn = rot_bn0 + 3.1415926;
//     // }
//     // else
//     // {
//     //     rot_bn = rot_bn0;
//     // }
//     rot_bn = rot_bn0;
//     return rot_bn;
// }

double sign(double p1[], double p2[], double p3[])
{
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
}


bool PointInTriangle(double pt[], double v1[], double v2[], double v3[])
{
    double d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}
