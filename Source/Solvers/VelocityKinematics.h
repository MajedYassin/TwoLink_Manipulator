
#ifndef TWOLINK_MANIP_VELOCITYKINEMATICS_H
#define TWOLINK_MANIP_VELOCITYKINEMATICS_H

#include "../Bot/Sbot.h"
#include "../Bot/State.h"
#include <map>
#include <vector>
#include <functional>


struct VelKinematics {

    VelKinematics(SBot& parameters, State& state);


    Eigen::Matrix2d get_jacobian(Eigen::Vector2d& q, int link);


    Eigen::Vector2d get_endeffector_velocity(Eigen::Vector2d& joint_velocity);


    std::vector<Eigen::Vector2d> end_effector_velocities(std::vector<Eigen::Vector2d>& position_array, std::vector<Eigen::Vector2d>& joint_velocity);

private:
    SBot bot;
    State instance;
    enum Direction {X, Y};
    std::map<const Direction, std::function<Eigen::Matrix2d
            (Eigen::Matrix2d &, Eigen::Vector2d &, int)>> Component;

};

#endif //TWOLINK_MANIP_VELOCITYKINEMATICS_H
