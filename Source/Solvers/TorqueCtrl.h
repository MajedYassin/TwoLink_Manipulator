
#ifndef TWOLINK_MANIP_TORQUECTRL_H
#define TWOLINK_MANIP_TORQUECTRL_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"

struct Dynamics
{
    explicit Dynamics(State& input);

    Eigen::Matrix2d inertia_matrix();

    Eigen::Vector2d get_gravity();

    Eigen::Vector2d get_coriolis();

private:
    State& state;
};




#endif //TWOLINK_MANIP_TORQUECTRL_H
