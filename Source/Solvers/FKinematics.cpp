#include "FKinematics.h"
#include "../Common/Common.h"
#include <vector>

//The Forward Kinematics Function Calculates the End-effector Pose of Two Link Manipulator
//Based onLink Parameters provided from the SBot struct and the Execution Interface which calls the FKin function below
//The function gives the end-effector pose for the joint angles q1b and q2b obtained from the OpInt struct



Eigen::Matrix3d ForwardKinematics::f_kin(SBot& bot, State& s, Eigen::Vector2d& qf)
{
    //Final Pose of End-Effector given the input joint angles
    Eigen::Matrix3d endpose = Eigen::Matrix3d::Identity();

    //T: Vector of n Translation Matrices; R: Vector of n Rotational Matrices;
    std::vector<Eigen::Matrix3d> R, T;
    R.clear();
    T.clear();


    Eigen::Vector2d dq;
    int n = 0; //number of links

    //Rot and Transl functions located in Common.cpp
    //Iterates the Rotations and Translations of reference frame for n number of links to find end-effector pose
    while( n != qf.size()) {
        for (int i = 0; i != qf.size(); ++i) {
            dq(i) = qf(i) - s.q(i); //Desired joint rotation (final - initial)
            R.emplace_back(Eigen::Matrix3d::Identity());
            R.back().block<2,2>(0, 0) = Rot(dq(i)); //function Rot returns a 2x2 matrix
            T.emplace_back(Transl(bot.link_length(i), bot.joint_displaced(i)));
        }
        (endpose *= R[n] ) * T[n];
        ++n;
    }

    return endpose;
}

