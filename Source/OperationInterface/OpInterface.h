
#ifndef TWOLINK_MANIP_OPINTERFACE_H
#define TWOLINK_MANIP_OPINTERFACE_H

#include <Eigen/Dense>
#include "../Bot/Sbot.h"
#include "../Solvers/TrpzTrajectory.h"
#include "../Solvers/TorqueCtrl.h"

//The Operation Interface interacts with the Solvers, acting as an interface through which the functions of the library can be called
//Solvers operations depend on the following solvers:
//The Inverse Kinematics, Forward Kinematics, Velocity Kinematics, Torque, state response;
//The Interface takes bot parameters from SBot, and current manipulator state from State as arguments;



struct OpInt
{
    State s;
    TrapezTrajectory trajectory;
    TorqueController torque;
    Eigen::MatrixX2d Q;
    SBot sbot;


    explicit OpInt(SBot& bot, State& instance) : trajectory(bot, instance), torque(instance, bot), s(instance), sbot(bot)
    {
        //Q = Eigen::MatrixX2d::Zero();
    }

    explicit OpInt(SBot& bot, Eigen::Vector2d& initial_position) : s(initial_position), trajectory(bot, s),
                    torque(s, bot), sbot(bot)
    {
        //Q = Eigen::MatrixX2d::Zero();
    }


    //Torque Control function calls
    std::vector<Eigen::Vector2d> get_feedforward_torque(Eigen::Vector2d& end_position);


    std::vector<Eigen::Vector2d> get_trajectory(Eigen::Vector2d& endq);


    //Forward and Inverse Kinematics function calls
    Eigen::Matrix3d find_pose(Eigen::Vector2d& end_position);


    Eigen::Vector2d find_joint_angles(Eigen::Matrix3d& pose);

};

#endif //TWOLINK_MANIP_OPINTERFACE_H
