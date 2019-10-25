
#ifndef TWOLINK_MANIP_OPINTERFACE_H
#define TWOLINK_MANIP_OPINTERFACE_H

#include <Eigen/Dense>
#include "../Bot/Sbot.h"
#include "../Solvers/TrpzTrajectory.h"
#include "../Solvers/TorqueCtrl.h"

//The OperationInterface computes the Inital and final properties of the model e.g. pose, and
//calls the functions of the Solver to execute the calculations necessary to compute:
//The Inverse Kinematics, Forward Kinematics or Required Torque of the Two Link Robot
//The Interface runs based on parameters specified in the bot parameters struct, SBot.



struct OpInt
{
    //Operations
    State s;
    TrapezTrajectory trajectory;
    InvDynamics torque;
    Eigen::MatrixX2d Q;
    SBot sbot;
    Eigen::Vector2d set_position;


    explicit OpInt(State& instance, TrapezTrajectory& traj, Dynamics& trqcontrol, Eigen::Vector2d& joint_configuration, SBot& bot) : trajectory{traj}, torque{}, s(instance), sbot(bot)
    {
        Q = Eigen::MatrixX2d::Zero();
        set_position = joint_configuration;
    }

    //Torque Control Solver Call function
    //Eigen::Matrix2Xd get_feedforward_torque();
    Eigen::Matrix2Xd get_feedforward_torque()
    {
        Eigen::MatrixX2d position_traj, velocity_traj, acceleration_traj;

        position_traj     = trajectory.tr_traj(s.q, set_position);
        velocity_traj     = trajectory.vel_traj(s.q, set_position);
        acceleration_traj = trajectory.acc_traj(s.q, set_position);

        return torque.feedforward_torque(acceleration_traj, velocity_traj, position_traj);
    }



    // Eigen::MatrixX2d get_trajectory(Eigen::Vector2d& endq);
    Eigen::MatrixX2d get_trajectory(Eigen::Vector2d& endq){
        return trajectory.tr_traj(s.q, endq);
    }

};

#endif //TWOLINK_MANIP_OPINTERFACE_H
