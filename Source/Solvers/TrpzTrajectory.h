
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../Bot/Sbot.h"
#include <Eigen/Dense>
#include "../Common/Common.h"
#include <cmath>
#include <map>
#include <algorithm>
#include <functional>
#include <memory>


struct TrapezTrajectory{

public:
    std::unique_ptr<Eigen::MatrixX2d> q_traj;
    std::unique_ptr<Eigen::MatrixX2d> qd_traj;
    std::unique_ptr<Eigen::MatrixX2d> qdd_traj;

    explicit TrapezTrajectory(SBot& sbot);

    void prioritise();


    void joint_acceleration();


    void acc_time();


    void duration();


    Eigen::MatrixX2d tr_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixX2d velocity_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixX2d acc_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


private:
    SBot& bot;
    double hmax;
    double Vmax;
    double t;
    double dt;
    int x; //iterator
    enum Phase {acc_phase, const_velocity, decel_phase};
    enum Time {a, d};
    std::map<const Time, double> T;
    std::map<const Phase, std::function<Eigen::MatrixX2d
            (Eigen::MatrixX2d, Eigen::Vector2d, std::map<const Time, double>, Eigen::Vector2d)>> map;
    Eigen::Vector2d h;
    Eigen::Vector2d A;
    Eigen::MatrixX2d Qa;
};

#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
