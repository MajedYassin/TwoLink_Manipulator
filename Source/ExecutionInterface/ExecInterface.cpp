#include "ExecInterface.h"

//The ExecutionInterface computes the Inital and final properties of the model e.g. pose, and
//calls the functions of the Solver to execute the calculations necessary to compute:
//The Inverse Kinematics, Forward Kinematics or Required Torque of the Two Link Robot
//The Interface runs based on parameters specified in the bot parameters struct, s_bot.

void ExecInt()
{
    //Eigen::Matrix3d Joint1_Pose;
    //Eigen::Matrix3d End_Pose;
    Eigen::Matrix3d Inert_M;
    Eigen::Matrix<double, 2, 1> Grav_M;
    Eigen::Matrix3d Cor_M;








    if(Task == "Inverse Kinematics") finq1; finq2; //finq1 = InvK[1]; finq1 = InvK[2];
    if(Task == "Forward Kinematics" || Task == "Torque") finq1 = qf[1]; finq2 = qf[2];
}
