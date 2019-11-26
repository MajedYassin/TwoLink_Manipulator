
#ifndef TWOLINK_MANIP_STATE_H
#define TWOLINK_MANIP_STATE_H

#include <Eigen/Dense>

struct State{
    Eigen::Vector2d q;
    Eigen::Vector2d qd;
    Eigen::Vector2d qdd;

    State() {
        q   = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
        qd  = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
        qdd = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
    }

    State(Eigen::Vector2d& q0){
        q = q0;
        qd  = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
        qdd = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
    }


    void set_position(const Eigen::Vector2d& position){
        q = position;
    }


    void set_velocity(const Eigen::Vector2d& velocity){
        qd = velocity;
    }


    void set_acceleration(const Eigen::Vector2d& acceleration){
        qdd = acceleration;
    }

};


#endif //TWOLINK_MANIP_STATE_H
