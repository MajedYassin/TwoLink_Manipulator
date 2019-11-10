
#ifndef TWOLINK_MANIP_TORQUECTRL_H
#define TWOLINK_MANIP_TORQUECTRL_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"
#include <vector>
#include <memory>
#include <map>

struct Dynamics
{
    double g;
    double dt;
    SBot& bot;
    State& s;
    Eigen::Vector2d link_length, link_cm, Iq;
    Eigen::Matrix2d Rq;

    explicit Dynamics(State& state, SBot& sbot);


    Eigen::Vector2d get_torque(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& qdd);


    Eigen::Matrix2d get_inertia_matrix(Eigen::Vector2d& q, Eigen::Vector2d& gravity);


    Eigen::Vector2d get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& gravity);


    Eigen::Vector2d get_gravity_vector(Eigen::Vector2d& q);


private:

    virtual Eigen::Matrix2d get_gravity(Eigen::Vector2d& q); //component of gravity used in get_torque forward recursion

};

struct TorqueController : public Dynamics{

    //std::vector<Eigen::Vector2d> acceleration_response, position_response_array,gravity_array, coriolis_array;
    //std::vector<Eigen::Matrix2d> inertia_array;

    explicit TorqueController(State& state, SBot& sbot) : Dynamics(state, sbot) {
        Kp = 0.0;
        Kv = 0.0;
    }


    std::vector<Eigen::Vector2d> feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj, std::vector<Eigen::Vector2d>& vel_traj, std::vector<Eigen::Vector2d>& acc_traj);


private:
    double Kp, Kv;

    static Eigen::Vector2d state_response(Eigen::Vector2d& torque, Eigen::Matrix2d& inertia, Eigen::Vector2d& coriolis,
                                          Eigen::Vector2d& gravity);
};




#endif //TWOLINK_MANIP_TORQUECTRL_H
