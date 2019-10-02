
#ifndef TWOLINK_MANIP_SBOT_H
#define TWOLINK_MANIP_SBOT_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

//The s_bot struct defines the fundamental properties and parameters of the robot manipulator
//This struct will be used as the base model from which to execute the computations in Execution Interface

struct s_bot
{
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

    //Inertia Terms
    double Inertia1;
    double Inertia2;



    s_bot(bool Default, double& Link1, double& Link2, double& Link1_cm, double& Link2_cm, double& mass1, double& mass2,
          std::vector<int>& q0, std::vector<int>& qf)

    {
        if(Default)
        {
            a1 = 0.2;
            a2 = 0.2;
            cm1 = 0.1;
            cm2 = 0.1;
            m1 = 1.0;
            m2 = 1.0;
            inq1 = 0;
            inq2 = 0;
            Inertia1 = (1/12) * a1* pow(0.05, 3);
            Inertia2 = Inertia1;
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
        //Will be computed or defined in the Operation Execution - Next Step
        finq1 = 0;
        finq2 = 0;
        Base_Pose = Eigen::Matrix3d(3, 3, 0);

    }

};


#endif //TWOLINK_MANIP_SBOT_H
