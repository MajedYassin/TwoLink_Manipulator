#include "FKinematics.h"

//The Forward Kinematics Function Calculates the End-effector Pose of Two Link Manipulator
//Based onLink Parameters provided from the s_bot struct and the Execution Interface which calls the FKin function below
//The function gives the end-effector pose for the joint angles q1b and q2b obtained from the ExecInt struct

Eigen::Matrix3d FKin(s_bot& Bot, ExecInt& Input)
{
    double q1a = Bot.Inq1;
    double q2a = Bot.Inq2;
    Eigen::Matrix3d BPose = s_bot.Base_Pose;

    //Final Pose of End-Effector give nby input joint angles
    Eigen::Matrix3d EPose;




}

