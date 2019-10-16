
#include "TorqueCtrl.h"

//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//The solver will comprise the Dynamics and Inverse Dynamics meber functions

Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    Rq = Eigen::Rotation2Dd(s.q);
    Iq << bot.Inertia1, bot.Inertia2;
    //Inertia of link with respect ot base frame: I = Rq * Eigen::Vector2d(sbot.Inertia1, sbot.Inertia2) * Rq.transpose();
    Omega << 0.0, 0.0;
    linear_acc1 << 0.0, 0.0;
    linear_acc2 << 0.0, 0.0;
    vel << 0.0, 0.0;
    ang_acc << 0.0, 0.0;
    l << bot.l1_length, bot.l2_length;
    link_cm << bot.cm1, bot.cm2;
    g = 9.81;
}

//Eigen::Vector2d TorqueOut(TorqueInt& Tau, Eigen::Vector2d& Dynamics){}


void Dynamics::forward_recursion_1(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::Vector2d Ac;

    //Linear acceleration : components of acceleration in x and y with respect to link frame
    Ac << qd(0) * qd(0) * link_cm(0), - qdd(0) * link_cm(0);
    linear_acc1 = Ac;

    Eigen::Matrix2d Rq1_T = Rot(q(0)).transpose();
    Gravity.col(0) = Rq1_T.col(1) * g;
}


void Dynamics::forward_recursion_2(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::Vector2d Ac1;
    Eigen::Vector2d Ac2;
    Eigen::Matrix2d Rq2 = Rot(q(1));

    //Linear acceleration : components of acceleration in x and y with respect to link frame
    Ac1 = (Rq2.transpose() * linear_acc1);
    Ac2 << (pow(qd(0) + qd(1), 2) * link_cm(1), (qdd(0) + qdd(1)) * link_cm(1));
    linear_acc2 = Ac1 - Ac2;

    double q2 = q(0) + q(1);
    Eigen::Matrix2d Rq2_T = Rot(q2).transpose();
    Gravity.col(1)= Rq2_T.col(1) * g;
}


void backward_recursion_2()
{
    
}


void backward_recursion_1()
{

}

