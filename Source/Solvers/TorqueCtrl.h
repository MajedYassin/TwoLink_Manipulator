
#ifndef TWOLINK_MANIP_TORQUECTRL_H
#define TWOLINK_MANIP_TORQUECTRL_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"
#include <memory>
#include <map>
#include <functional>

struct Dynamics
{
    Eigen::Matrix2d Rq;
    Eigen::Matrix2d Inertia;
    Eigen::Matrix2d Gravity;
    Eigen::Matrix2d Coriolis;
    std::unique_ptr<Eigen::Matrix2Xd> Torque;

    explicit Dynamics(State& state, SBot& sbot);


    Eigen::MatrixXd forward_recursion(Eigen::VectorXd& qdd, Eigen::VectorXd& qd, Eigen::VectorXd& q);


    Eigen::VectorXd backward_recursion(Eigen::VectorXd& qdd, Eigen::VectorXd& qd, Eigen::VectorXd& q, Eigen::Matrix2Xd& linear_acc);


    static Eigen::Matrix2d inertia_tensor(Eigen::Vector2d& I);


    Eigen::MatrixXd get_inertia_matrix(Eigen::VectorXd& q);


   // void get_coriolis_matrix();


    Eigen::VectorXd get_gravity(Eigen::VectorXd& q);


    Eigen::MatrixXd get_jacobian(Eigen::VectorXd& q);

    Eigen::Matrix2Xd get_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj);


private:
    State& s;
    SBot& bot;
    Eigen::Vector2d l, link_cm, Iq;
    double g;
    // enum Link { a, b, e };
    //Jacobian variables
    enum Direction {X , Y};
    std::map<const Direction, std::function<Eigen::MatrixXd
            (Eigen::MatrixXd, Eigen::VectorXd)>> Component;
};




#endif //TWOLINK_MANIP_TORQUECTRL_H
