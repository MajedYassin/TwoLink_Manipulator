
#ifndef TWOLINK_MANIP_STATE_H
#define TWOLINK_MANIP_STATE_H

#include <Eigen/Dense>

struct State{
    Eigen::Vector2d q;
    Eigen::Vector2d dq;
    Eigen::Vector2d ddq;

    State() {
        q   = (Eigen::Vector2d(2, 1) << 0, 0).finished();
        dq  = (Eigen::Vector2d(2, 1) << 0, 0).finished();
        ddq = (Eigen::Vector2d(2, 1) << 0, 0).finished();
    }


    void set_position(const Eigen::Vector2d& position){
        q = position;
    }


    void set_velocity(const Eigen::Vector2d& velocity){
        dq = velocity;
    }


    void set_acceleration(const Eigen::Vector2d& acceleration){
        dq = acceleration;
    }

};


#endif //TWOLINK_MANIP_STATE_H
