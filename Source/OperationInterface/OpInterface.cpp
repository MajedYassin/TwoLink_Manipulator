
#include "OpInterface.h"
#include "../Solvers/InvKinematics.h"
#include "../Solvers/FKinematics.h"


OpInt::OpInt(SBot& bot, State& instance) : trajectory(bot, instance), torque(instance, bot), s(instance), sbot(bot){

}

OpInt::OpInt(SBot& bot, Eigen::Vector2d& initial_position) : s(initial_position), trajectory(bot, s),
        torque(s, bot), sbot(bot){

}


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


Eigen::Vector2d OpInt::pose_to_angles(Eigen::Matrix3d& pose)
{
    return inv_kin(sbot, pose);
}


Eigen::Vector2d OpInt::cartesian_to_angles(double x, double y)
{
    return joint_angles(sbot, x, y);
}

