
#ifndef TWOLINK_MANIP_SBOT_H
#define TWOLINK_MANIP_SBOT_H

#include <Eigen/Dense>

//The SBot struct defines the fundamental properties and parameters of the robot manipulator
//This struct will be used as the base model from which to execute the computations in Execution Interface

struct SBot
{
    //Link Lengths
    Eigen::Vector2d link_length, link_cm, mass, link_inertia, joint_displaced;
    //Inertia Terms
    double Inertia1 = 0.0625;
    double Inertia2 = Inertia1;
    //Base Pose at Origin(0, 0)
    Eigen::Matrix3d StartPose = Eigen::Matrix3d::Identity();
    //Motor Parameters
    double Amax;
    double Vmax;

    SBot();

    void set_linklengths(double a, double b);


    void set_linkmasses(double a);


    void set_acceleration(double A);


    void set_velocity(double V);

};


#endif //TWOLINK_MANIP_SBOT_H
