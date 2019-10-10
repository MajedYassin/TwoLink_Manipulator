
#ifndef TWOLINK_MANIP_EXECINTERFACE_H
#define TWOLINK_MANIP_EXECINTERFACE_H

#include <Eigen/Dense>
#include "../Common/Common.h"
#include "../Bot/Sbot.h"
#include "../Solvers/TrpzTrajectory.h"

//The ExecutionInterface computes the Inital and final properties of the model e.g. pose, and
//calls the functions of the Solver to execute the calculations necessary to compute:
//The Inverse Kinematics, Forward Kinematics or Required Torque of the Two Link Robot
//The Interface runs based on parameters specified in the bot parameters struct, SBot.



struct ExecInt
{
    //Operations
    State s;
    TrapezTrajectory trajectory;
    Eigen::MatrixX2d Q;
    SBot sbot;

    Eigen::Vector2d finq;

    ExecInt(State& instance, TrapezTrajectory& traj, Eigen::Vector2d& joint_configuration, SBot& bot) : trajectory(traj), s(instance), sbot(bot)
    {
        finq = joint_configuration;
        Q = Eigen::MatrixX2d::Zero();
    }


    // Eigen::MatrixX2d get_trajectory(Eigen::Vector2d& endq);

    Eigen::MatrixX2d get_trajectory(Eigen::Vector2d& endq){
        return trajectory.tr_traj(s.q, endq);
    }

};

#endif //TWOLINK_MANIP_EXECINTERFACE_H
