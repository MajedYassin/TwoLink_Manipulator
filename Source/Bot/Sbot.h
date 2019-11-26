
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
    double Inertia1 = 0.125;
    double Inertia2 = Inertia1;
    //Base Pose at Origin(0, 0)
    Eigen::Matrix3d StartPose = Eigen::Matrix3d::Identity();
    //Motor Parameters
    double Amax;
    double Vmax;

    SBot(){
        link_length = (Eigen::Vector2d(2) << 0.5, 0.5).finished();
        link_cm = (Eigen::Vector2d(2) << 0.25, 0.25).finished();
        mass = (Eigen::Vector2d(2) << 2.0, 2.0).finished();
        link_inertia = (Eigen::Vector2d(2, 1) << Inertia1, Inertia2).finished();
        joint_displaced = (Eigen::Vector2d(2) << 0.0, 0.0).finished();
        Amax = 8.0;
        Vmax = 10.0;
    }

    void set_linklengths(double a, double b);


    void set_linkmasses(double a);


    void set_acceleration(double A);


    void set_velocity(double V);

};


#endif //TWOLINK_MANIP_SBOT_H
