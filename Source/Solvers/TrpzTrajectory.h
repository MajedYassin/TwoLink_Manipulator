
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../Bot/Sbot.h"
#include "../Bot/State.h"
#include <Eigen/Dense>
#include "../Common/Common.h"
#include <cmath>
#include <map>
#include <algorithm>
#include <functional>
#include <memory>
#include <gtest/gtest.h>



struct TrapezTrajectory{

public:
    std::unique_ptr<Eigen::MatrixXd> q_traj;
    std::unique_ptr<Eigen::MatrixXd> qd_traj;
    std::unique_ptr<Eigen::MatrixXd> qdd_traj;

    explicit TrapezTrajectory(SBot& sbot, State& s);

    void prioritise();


    void joint_acceleration();


    void acc_time();


    void duration();


    Eigen::MatrixXd tr_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixXd vel_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixXd acc_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


private:
    SBot& bot;
    State s;
    double hmax;
    double Vmax;
    double t;
    double dt;
    int x; //iterator
    enum Phase {acc_phase, const_velocity, decel_phase};
    enum Time {a, d};
    std::map<const Time, double> T;
    std::map<const Phase, std::function<Eigen::MatrixX2d
            (Eigen::MatrixXd, Eigen::Vector2d, std::map<const Time, double>, Eigen::Vector2d)>> map;
    Eigen::Vector2d h;
    Eigen::Vector2d A;
};

#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
