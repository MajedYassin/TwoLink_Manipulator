#include <Eigen/Dense>
#include <vector>

//The ExecutionInterface computes the Inital and final properties of the model e.g. pose, and
//calls the functions of the Solver to execute the calculations necessary to compute:
//The Inverse Kinematics, Forward Kinematics or Required Torque of the Two Link Robot
//The Interface runs based on parameters specified in the bot parameters struct, s_bot.

struct ExecInt
{
    Eigen::Matrix3d Base;
    Eigen::Matrix3d Joint1_Pose;
    Eigen::Matrix3d End_Pose;
    Eigen::Matrix3d InertiaMatrix;
    Eigen::Matrix<double, 2, 1> GravityMatrix;
    Eigen::Matrix3d CoriolisMatrix;

    //Operations



    ExecInt(s_bot& Bot, std::string& )
    {
        Base = Bot.Base_Pose;



    }




    if(Task == "Inverse Kinematics") finq1; finq2; //finq1 = InvK[1]; finq1 = InvK[2];
    if(Task == "Forward Kinematics" || Task == "Torque") finq1 = qf[1]; finq2 = qf[2];
};
