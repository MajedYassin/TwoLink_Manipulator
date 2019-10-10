
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/State.h"
#include <Eigen/Dense>
#include "../Common/Common.h"
#include <cmath>
#include <map>
#include <algorithm>
#include <functional>


struct TrapezTrajectory{
private:
    SBot& bot;
    double hmax;
    double Vmax;
    double dt;
    int x; //iterator
    enum Phase {acc_phase, const_velocity, decel_phase};
    enum Time {a, d};
    std::map<const Time, double> T;
    std::map<const Phase, std::function<Eigen::MatrixX2d
    (Eigen::MatrixX2d, Eigen::Vector2d, std::map<const Time, double>, Eigen::Vector2d)>> map;
    Eigen::Array2d h;
    Eigen::Array2d A;
    Eigen::MatrixX2d Qa;

public:
    Eigen::MatrixX2d q_traj;
    Eigen::MatrixX2d qd_traj;
    Eigen::MatrixX2d qdd_traj;

    TrapezTrajectory(SBot& sbot);

    void prioritise();


    void joint_acceleration();


    void acc_time();


    void duration();


    Eigen::MatrixX2d tr_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixX2d velocity_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    Eigen::MatrixX2d acc_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);
};

#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
