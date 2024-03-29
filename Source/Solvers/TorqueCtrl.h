
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
    double g, dt;
    SBot& bot;
    State& s;
    Eigen::Vector2d link_length, link_cm, Iq;
    Eigen::Matrix2d Rq;
    std::vector<Eigen::Vector2d> accelerations;
    double friction_coefficient;


    explicit Dynamics(State& state, SBot& sbot);


    Eigen::Vector2d get_torque(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& qdd);


    Eigen::Matrix2d get_inertia_matrix(Eigen::Vector2d& q, Eigen::Vector2d& gravity);


    Eigen::Vector2d get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& gravity);


    Eigen::Vector2d get_gravity_vector(Eigen::Vector2d& q);


    Eigen::Vector2d get_friction(Eigen::Vector2d& qd);


    Eigen::Matrix2d get_gravity(Eigen::Vector2d& q); //component of gravity used in get_torque forward recursion

};

struct TorqueController : public Dynamics{

    std::vector<Eigen::Vector2d> position_array, velocity_array, acceleration_array;

    explicit TorqueController(State& state, SBot& sbot) : Dynamics(state, sbot) {
        Kp = 0.0;
        Kv = 0.0;
    }


    std::vector<Eigen::Vector2d> feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj, std::vector<Eigen::Vector2d>& vel_traj,
                                                    std::vector<Eigen::Vector2d>& acc_traj);


private:
    double Kp, Kv;

    //Eigen::Vector2d get_gravity_vec(Eigen::Vector2d& q);

    static Eigen::Vector2d forward_dynamics(Eigen::Vector2d& torque, Eigen::Matrix2d& inertia, Eigen::Vector2d& coriolis,
                                            Eigen::Vector2d& gravity, Eigen::Vector2d& friction);
};



struct PendulumModel : public Dynamics {

    std::vector<Eigen::Vector2d> position_array, velocity_array, acceleration_array;

    PendulumModel(State& state, SBot& sbot) : Dynamics(state, sbot){
    }

    void release_pendulum();

};


#endif //TWOLINK_MANIP_TORQUECTRL_H
