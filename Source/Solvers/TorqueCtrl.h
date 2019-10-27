
#ifndef TWOLINK_MANIP_TORQUECTRL_H
#define TWOLINK_MANIP_TORQUECTRL_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"
#include <vector>
#include <memory>
#include <map>
#include <functional>

struct Dynamics
{
    double g;
    double dt;
    SBot& bot;
    State& s;
    Eigen::Vector2d link_length, link_cm, Iq;
    Eigen::Matrix2d Rq;

    explicit Dynamics(State& state, SBot& sbot);


    Eigen::Matrix2d forward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q);


    Eigen::Vector2d backward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q, Eigen::Matrix2d& linear_acc);



    virtual Eigen::Vector2d get_gravity(Eigen::Vector2d& q);

private:
    //Jacobian variables
    enum Direction {X , Y};
    std::map<const Direction, std::function <Eigen::Matrix2d
            (Eigen::Matrix2d, Eigen::Vector2d, int)>> Component;
};

struct InvDynamics : public Dynamics{

    InvDynamics() = default;


    //Eigen::Matrix2d inertia_tensor(Eigen::Vector2d& I);


    Eigen::Vector2d get_inertia_vector(Eigen::Vector2d& q, Eigen::Vector2d& qdd, Eigen::Vector2d gravity);


    //Eigen::Matrix2d get_inertia_matrix(Eigen::Vector2d& q);


    Eigen::Vector2d get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& qdd, Eigen::Vector2d& gravity, Eigen::Vector2d& inertia);


    //Eigen::Matrix2d get_coriolis_matrix(Eigen::Vector2d& q, Eigen::Vector2d& q0, Eigen::Vector2d& qd, Eigen::Matrix2d& M);


    //Eigen::Matrix2d get_jacobian(Eigen::Vector2d& q, int link);


    //Eigen::Vector2d get_gravity(Eigen::Vector2d& q) override;


    Eigen::Vector2d get_gravity_vector(Eigen::Vector2d& q);


    Eigen::Vector2d state_response(Eigen::Vector2d& torque, Eigen::Vector2d& inertia, Eigen::Vector2d& coriolis,
                                    Eigen::Vector2d& gravity);


    Eigen::Matrix2Xd feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj, std::vector<Eigen::Vector2d>& vel_traj, std::vector<Eigen::Vector2d>& acc_traj);


private:
    double Kp = 20;
    double Kv = 5;
    //double Kd = 1;

    //Jacobian variables
    enum Direction {X , Y};
    std::map<const Direction, std::function <Eigen::Matrix2d
            (Eigen::Matrix2d, Eigen::Vector2d, int)>> Component;

};




#endif //TWOLINK_MANIP_TORQUECTRL_H
