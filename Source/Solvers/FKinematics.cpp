#include "FKinematics.h"

//The Forward Kinematics Function Calculates the End-effector Pose of Two Link Manipulator
//Based onLink Parameters provided from the s_bot struct and the Execution Interface which calls the FKin function below
//The function gives the end-effector pose for the joint angles q1b and q2b obtained from the ExecInt struct

Eigen::Matrix3d FKin(s_bot& Bot, ExecInt& Input)
{
    double q1a = Bot.inq1;
    double q2a = Bot.inq2;
    Eigen::Matrix3d BPose = Bot.Base_Pose;
    //Desired Joint  Angles set in Ecexution Interface - still need to add in struct
    double q1b = ExecInt.Inputq1;
    double q2b = ExecInt.Inputq2;


    //Translation Matrices - Maybe Move to Common functions.cpp
    double a1 = Bot.a1;
    double d1 = 0.0;
    double a2 = Bot.a2;
    double d2 = 0.0;
    Eigen::Matrix3d T1, T2;
    T1 = Transl(a1, d1);
    T2 = Transl(a2, d2);

    //Rotation Matrix - Function Rot (Rotation Matrix function of q) will be in Common
    Eigen::Matrix3d R1 = Rot(q1b);
    Eigen::Matrix3d R2 = Rot(q2b);

    //Final Pose of End-Effector give nby input joint angles
    Eigen::Matrix3d EPose;

    EPose = ((R1*T1)*R2)*T2;

    return EPose;
}

