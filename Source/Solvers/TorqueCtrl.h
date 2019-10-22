
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
    double g;
    double dt;
    SBot& bot;
    Eigen::Vector2d link_length, link_cm, Iq;
    Eigen::Matrix2d Rq;
    std::unique_ptr<Eigen::Matrix2Xd> Torque;

    explicit Dynamics(State& state, SBot& sbot);


    Eigen::MatrixXd forward_recursion(Eigen::VectorXd& qdd, Eigen::VectorXd& qd, Eigen::VectorXd& q);


    Eigen::VectorXd backward_recursion(Eigen::VectorXd& qdd, Eigen::VectorXd& qd, Eigen::VectorXd& q, Eigen::Matrix2Xd& linear_acc);


    static Eigen::Matrix2d inertia_tensor(Eigen::Vector2d& I);


    Eigen::MatrixXd get_inertia_matrix(Eigen::VectorXd& q);


   // void get_coriolis_matrix();


    virtual Eigen::VectorXd get_gravity(Eigen::VectorXd& q);


    Eigen::MatrixXd get_jacobian(Eigen::VectorXd& q, int link);

    Eigen::Matrix2Xd get_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj);


private:
    State& s;
    //Jacobian variables
    enum Direction {X , Y};
    std::map<const Direction, std::function <Eigen::MatrixXd
            (Eigen::MatrixXd, Eigen::VectorXd, int)>> Component;
};

struct InvDynamics : public Dynamics{

    InvDynamics();


    Eigen::VectorXd get_gravity(Eigen::VectorXd& q) override;


    Eigen::MatrixX3d state_response(Eigen::VectorXd& q_des, Eigen::VectorXd& qd_des, Eigen::MatrixX3d& response,
                                    Eigen::VectorXd& torque, Eigen::MatrixXd& inertia, Eigen::MatrixXd& coriolis,
                                    Eigen::VectorXd& gravity);


    Eigen::MatrixXd feedforward_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj);


private:
    double Kv, Kp, Kd;

};




#endif //TWOLINK_MANIP_TORQUECTRL_H
