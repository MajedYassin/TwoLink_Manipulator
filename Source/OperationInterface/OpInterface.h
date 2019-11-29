
#ifndef TWOLINK_MANIP_OPINTERFACE_H
#define TWOLINK_MANIP_OPINTERFACE_H

#include <Eigen/Dense>
#include "../Bot/Sbot.h"
#include "../Solvers/TrpzTrajectory.h"
#include "../Solvers/TorqueCtrl.h"

//The Operation Interface interacts with the Solvers, acting as an interface through which the functions can be called
//List of supported solvers:
//The Inverse Kinematics, Forward Kinematics, Velocity Kinematics, Torque, state response;
//The Interface constructor takes bot parameters from a SBot object and current state from a State object as arguments;



struct OpInt
{
    State s;
    TrapezTrajectory trajectory;
    TorqueController torque;
    SBot sbot;


    explicit OpInt(SBot& bot, State& instance);

    explicit OpInt(SBot& bot, Eigen::Vector2d& initial_position);


    //Torque Control functions
    std::vector<Eigen::Vector2d> get_feedforward_torque(Eigen::Vector2d& end_position);


    std::vector<Eigen::Vector2d> mimic_pendulum();

    //Trape Trajectory function
    std::vector<Eigen::Vector2d> get_trajectory(Eigen::Vector2d& endq);


    //Forward and Inverse Kinematics functions
    Eigen::Matrix3d find_pose(Eigen::Vector2d& end_position);


    Eigen::Vector2d pose_to_angles(Eigen::Matrix3d& pose);


    Eigen::Vector2d cartesian_to_angles(double x, double y);

};

#endif //TWOLINK_MANIP_OPINTERFACE_H
