
#include "OpInterface.h"
#include "../Solvers/InvKinematics.h"
#include "../Solvers/FKinematics.h"

std::vector<Eigen::Vector2d> OpInt::get_feedforward_torque(Eigen::Vector2d& end_position)
{
    std::vector<Eigen::Vector2d> position_traj, velocity_traj, acceleration_traj;

    position_traj     = trajectory.pos_traj(s.q, end_position);
    velocity_traj     = trajectory.vel_traj();
    acceleration_traj = trajectory.acc_traj();

    return torque.feedforward_torque(position_traj, velocity_traj, acceleration_traj);
}




std::vector<Eigen::Vector2d> OpInt::get_trajectory(Eigen::Vector2d& end_position)
{
    return trajectory.pos_traj(s.q, end_position);
}


Eigen::Matrix3d OpInt::find_pose(Eigen::Vector2d& end_position)
{
    return f_kin(sbot, s, end_position);
}


Eigen::Vector2d OpInt::find_joint_angles(Eigen::Matrix3d& pose)
{
    return inv_kin(sbot, pose);
}

Eigen::Vector2d OpInt::joint_rotation(Eigen::Vector2d& end_position)
{
    return
}

