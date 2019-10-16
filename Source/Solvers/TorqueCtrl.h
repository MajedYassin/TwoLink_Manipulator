
#ifndef TWOLINK_MANIP_TORQUECTRL_H
#define TWOLINK_MANIP_TORQUECTRL_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"

struct Dynamics
{
    Eigen::Matrix2d Rq;
    Eigen::Matrix2d Inertia;
    Eigen::Matrix2d Gravity;
    Eigen::Matrix2d Coriolis;
    Eigen::Vector2d Torque;

    explicit Dynamics(State& state, SBot& sbot);


    void forward_recursion_1(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q);


    void forward_recursion_2(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q);


    void backward_recursion_2(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q);


    void backward_recursion_1(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q);


    Eigen::ArrayX2d get_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj);


private:
    State& s;
    SBot& bot;
    Eigen::Vector2d Omega;
    Eigen::Vector2d linear_acc1;
    Eigen::Vector2d linear_acc2;
    Eigen::Vector2d vel;
    Eigen::Vector2d ang_acc;
    Eigen::Vector2d l;
    Eigen::Vector2d link_cm;
    Eigen::Vector2d Iq;
    Eigen::Vector2d force2;
    double g;
};




#endif //TWOLINK_MANIP_TORQUECTRL_H
