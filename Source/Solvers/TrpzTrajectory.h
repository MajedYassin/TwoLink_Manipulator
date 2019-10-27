
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
#include <vector>



struct TrapezTrajectory{

public:
    std::vector<Eigen::Vector2d> q_traj, qd_traj, qdd_traj;

    explicit TrapezTrajectory(SBot& sbot, State& s);

    void prioritise();


    void joint_acceleration();


    void acc_time();


    void duration();


    std::vector<Eigen::Vector2d> tr_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    std::vector<Eigen::Vector2d> pos_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf);


    std::vector<Eigen::Vector2d> vel_traj();


    std::vector<Eigen::Vector2d> acc_traj();


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
    std::map<const Phase, std::function< std::vector<Eigen::Vector2d>
            (std::vector<Eigen::Vector2d>& , Eigen::Vector2d& , std::map<const Time, double>, Eigen::Vector2d& )>> phase;
    Eigen::Vector2d h;
    Eigen::Vector2d A;
};

#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
