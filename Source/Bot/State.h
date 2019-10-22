
#ifndef TWOLINK_MANIP_STATE_H
#define TWOLINK_MANIP_STATE_H

#include <Eigen/Dense>

struct State{
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;

    State(int& links) {
        q   = Eigen::VectorXd::Zero(links, 1);
        qd  = Eigen::VectorXd::Zero(links, 1);
        qdd = Eigen::VectorXd::Zero(links, 1);
    }


    void set_position(const Eigen::VectorXd& position){
        q = position;
    }


    void set_velocity(const Eigen::VectorXd& velocity){
        qd = velocity;
    }


    void set_acceleration(const Eigen::VectorXd& acceleration){
        qdd = acceleration;
    }

};


#endif //TWOLINK_MANIP_STATE_H
