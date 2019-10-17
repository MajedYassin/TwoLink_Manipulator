
#ifndef TWOLINK_MANIP_SBOT_H
#define TWOLINK_MANIP_SBOT_H

#include <Eigen/Dense>

//The SBot struct defines the fundamental properties and parameters of the robot manipulator
//This struct will be used as the base model from which to execute the computations in Execution Interface

struct SBot
{
    //Link Lengths
    Eigen::Vector2d link_length, link_cm, mass, link_inertia;
    double l1_length = 0.2;
    double l2_length = 0.2;
    double cm1 = 0.1;
    double cm2 = 0.1;
    //Masses
    const double mass1 = 1.0;
    const double mass2 = 1.0;
    //Initial Joint Angles in radians
    const //Inertia Terms
    double Inertia1 = (1/12)*0.2*(pow(0.1, 3));
    double Inertia2 = Inertia1;
    //Base Pose at Origin(0, 0)
    Eigen::Matrix3d StartPose = Eigen::Matrix3d::Identity();
    //Motor Parameters
    double Amax;
    double Vmax;

    SBot(){
        link_length = (Eigen::Vector2d(2, 1) <<0.2, 0.2).finished();
        link_cm = (Eigen::Vector2d(2, 1) <<0.1, 0.1).finished();
        mass = (Eigen::Vector2d(2, 1) <<1.0, 1.0).finished();
        link_inertia = (Eigen::Vector2d(2, 1) << Inertia1, Inertia2).finished();
         Amax = 8.0;
         Vmax = 10.0;
    }
};


#endif //TWOLINK_MANIP_SBOT_H
