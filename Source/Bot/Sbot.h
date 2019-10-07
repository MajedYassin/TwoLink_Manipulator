
#ifndef TWOLINK_MANIP_SBOT_H
#define TWOLINK_MANIP_SBOT_H

#include <Eigen/Dense>

//The SBot struct defines the fundamental properties and parameters of the robot manipulator
//This struct will be used as the base model from which to execute the computations in Execution Interface

struct SBot
{
    //Link Lengths
    const double l1_length = 0.2;
    const double l2_length = 0.2;
    const double cm1 = 0.1;
    const double cm2 = 0.1;
    //Masses
    const double mass1 = 1.0;
    const double mass2 = 1.0;
    //Initial Joint Angles in radians
    const double inq1 = 0.0;
    const double inq2 = 0.0;
    const //Inertia Terms
    const double Inertia1 = (1/12)*0.2*(pow(0.1, 3));
    const double Inertia2 = Inertia1;
    //Base Pose at Origin(0, 0)
    Eigen::Matrix3d BasicPose = Eigen::Matrix3d::Identity();
    //Motor Parameters
    double Amax = 8.0;
    double Vmax = 10.0;

    SBot() = default;
};


#endif //TWOLINK_MANIP_SBOT_H
