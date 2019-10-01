#include "s_bot.h"


struct s_bot{
    //Link Lengths
    double a1;
    double a2;
    double cm1;
    double cm2;
    //Link Masses
    double m1;
    double m2;
    //Initial Joint Angles
    signed int inq1;
    signed int inq2;
    //desired/required joint angles
    signed int finq1;
    signed int finq2;
    //Matrices
    Eigen::Matrix3d Base_Pose;
    Eigen::Matrix3d Joint1_Pose;
    Eigen::Matrix3d End_Pose;
    Eigen::Matrix3d Inert_M;
    Eigen::Matrix<double, 2, 1> Grav_M;
    Eigen::Matrix3d Cor_M;

    //Coordinates


    //Inverse and Forward Kinematics


    //Dynamics





    s_bot(bool Default, std::string& Task,
            double& Link1, double& Link2, double& Link1_cm, double& Link2_cm, double& mass1, double& mass2,
            std::vector<int>& q0, std::vector<int>& qf )

    {
        if(Default){
            a1 = 0.2;
            a2 = 0.2;
            cm1 = 0.1;
            cm2 = 0.1;
            m1 = 1.0;
            m2 = 1.0;
            inq1 = 0;
            inq2 = 0;
        }
        else
        {
            a1 = Link1;
            a2 = Link2;
            cm1 = Link1_cm;
            cm2 = Link2_cm;
            m1 = mass1;
            m2 = mass2;
            inq1 = q0[1];
            inq2 = q0[2];
        }
        if(Task == "Inverse Kinematics") finq1; finq2; //finq1 = InvK[1]; finq1 = InvK[2];
        if(Task == "Forward Kinematics" || Task == "Torque") finq1 = qf[1]; finq2 = qf[2];

    }



};