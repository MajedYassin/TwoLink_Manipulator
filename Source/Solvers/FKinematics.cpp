#include "FKinematics.h"

//The Forward Kinematics Function Calculates the End-effector Pose of Two Link Manipulator
//Based onLink Parameters provided from the SBot struct and the Execution Interface which calls the FKin function below
//The function gives the end-effector pose for the joint angles q1b and q2b obtained from the OpInt struct



Eigen::Matrix3d ForwardKinematics::f_kin(SBot& bot, State& s, Eigen::Vector2d& qf)
{
    double q1a = s.q(0);
    double q2a = s.q(1);
    //Desired Joint  Angles set in Execution Interface - still need to add in struct

    //Translation Matrices - Maybe Move to Common functions.cpp
    double d1 = 0.0;
    double d2 = 0.0;
    Eigen::Matrix3d T1, T2;
    T1 = Transl(bot.link_length(0), d1);
    T2 = Transl(bot.link_length(1), d2);

    //qf: final joint configuration/angle
    //Rotation Matrix - Function Rot (Rotation Matrix function of q) will be in Common
    Eigen::Matrix3d R1 = Rot(qf(0));
    Eigen::Matrix3d R2 = Rot(qf(1));

    //Final Pose of End-Effector give nby input joint angles
    Eigen::Matrix3d EPose;

    EPose = ((R1*T1)*R2)*T2;

    return EPose;
}

