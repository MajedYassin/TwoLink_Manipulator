
#include "TorqueCtrl.h"

//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//The solver will comprise the Dynamics and Inverse Dynamics member functions

Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    Torque = std::make_unique<Eigen::Matrix2Xd>(2, 100);

    //robot manipulator parameters
    Iq << bot.link_inertia(0), bot.link_inertia(1);
    //Inertia of link with respect ot base frame: I = Rq * Eigen::Vector2d(sbot.Inertia1, sbot.Inertia2) * Rq.transpose();

    link_length << bot.link_length(0), bot.link_length(1);
    link_cm << bot.link_cm(0), bot.link_cm(1);
    g  = 9.81;
    dt = 0.01;

    //Jacobian Parameters
    Component[X] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double x = 0.0;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double dq = 0.0;
                while (m <= n) {
                    dq += q(m);
                    ++m;
                }
                if (m = l) x = link_cm(n) * sin(dq);
                else x = link_length(n) * sin(dq);
                Jac(0, i) = Jac(0, i) - x;
            }
        }
        return Jac;
    };
    Component[Y] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double x = 0;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double dq = 0.0;
                while(m <= n) {
                    dq += q(m);
                    ++m;
                }
                if (m = l) x = link_cm(n) * cos(dq);
                else x = link_length(n) * cos(dq);
                Jac(1, i) = Jac(1, i) + x;
            }

        }
        return Jac;
    };
}


//Forward recursion function for N-link manipulator
Eigen::Matrix2d Dynamics::forward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::Matrix2d Ac, Ae;
    int num = q.size();
    Ac = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at centre of mass
    Ae = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at end of link

    //Linear accelerations : components of acceleration in x and y with respect to individual link frame
    for(int n = 0; n != q.size(); ++n) {
        double qcd = 0.0; //sum of angular velocity components
        double qcdd = 0.0;  //sum of angular acceleration components
        int m = 0;  //joint component iterator
        while (m <= n) {
            qcd += qd(m);
            qcdd += qdd(m);
            ++m;
        }
        Eigen::Vector2d An_1, Ac_n, Ae_n;
        if (m > 0) An_1 = (Rot(q(m - 1))).transpose() * Ae.col(m - 1);
        else An_1 = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
        Ac_n = (Eigen::Vector2d(2, 1) << -1 * pow(qcd, 2) * link_cm(n), qcdd * link_cm(n)).finished();
        Ae_n = (Eigen::Vector2d(2, 1) << -1 * pow(qcd, 2) * link_cm(n), qcdd * link_length(n)).finished();
        Ae.col(n) = An_1 + Ae_n;
        Ac.col(n) = An_1 + Ac_n;
    }

    return Ac;
}

Eigen::Vector2d Dynamics::backward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q, Eigen::Matrix2d& linear_acc)
{
    //Base Link Torque
    Eigen::Vector2d force;
    Eigen::Vector2d gravity;
    gravity = get_gravity(q);
    Eigen::Vector2d torque;

    for(int n = q.size(); n >= 0; --n)
    {
        double qcdd = 0.0;
        signed int m = n;
        while(m >= 0){
            qcdd += q(m);
            --m;
        }
        if((m < 0)) {
            force(n) = bot.mass(n) * linear_acc(2, n) + gravity(n);
            torque(n) = Iq(n) * qcdd + force(n) * link_cm(n);
        }
        else {
            force(n) = bot.mass(n) * linear_acc(2, n) + Rot(q(n))(1, 1) * force(n + 1) - gravity(n);
            torque(n) = torque(n + 1) - force(n) * link_cm(n) - Rot(q(n))(1, 1) * force(n + 1) * link_length(n) + Iq(n) * qdd(n);
        }
    }
    return torque;
}

/*
Eigen::Matrix2Xd Dynamics::get_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj)
{
    Eigen::Matrix2Xd linear_acc;
    Eigen::Matrix2Xd torque;

    for(int i = 0; i != q_traj.rows(); ++i)
    {
        Eigen::VectorXd q, qd, qdd;
        qdd = (Eigen::Vector2d(2, 1) << qdd_traj(i, 0), qdd_traj(i, 1)).finished();
        qd  = (Eigen::Vector2d(2, 1) << qd_traj(i, 0), qd_traj(i, 1)).finished();
        q   = (Eigen::Vector2d(2, 1) << q_traj(i, 0), q_traj(i, 1)).finished();


        linear_acc = forward_recursion(qdd, qd, q);
        torque.col(i) = backward_recursion( qdd, qd, q, linear_acc);
    }

    return torque;
}
 */


Eigen::Matrix2Xd InvDynamics::feedforward_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj)
{
    Integrator vel_response(s.qd, dt);
    Integrator pos_response(s.q, dt);
    Eigen::Matrix2d linear_acc;
    Eigen::Matrix2Xd torque;
    Eigen::Vector2d q_response, qd_response, qdd_response, torque_i, gravity;
    Eigen::Vector2d q_error, qd_error, qdd_error;
    q_response   = Eigen::Vector2d::Zero();
    qd_response  = Eigen::Vector2d::Zero();
    qdd_response = Eigen::Vector2d::Zero();
    Eigen::Matrix2d inertia_matrix, coriolis_matrix;

    for(int i = 0; i != q_traj.rows(); ++i)
    {
        Eigen::Vector2d q, qd, qdd;
        qdd = qdd_traj.row(i);
        qd  = qd_traj.row(i);
        q   = q_traj.row(i);

        inertia_matrix = get_inertia_matrix(q);
        coriolis_matrix = get_coriolis_matrix(q, qd);
        gravity = get_gravity(q);


        q_error   = q   - q_response;
        qd_error  = qd  - qd_response;
        qdd_error = qdd - qdd_response;

        linear_acc = forward_recursion(qdd, qd, q);
        torque_i = backward_recursion( qdd, qd, q, linear_acc) + Kv * qd_error + Kp * q_error;

        torque.col(i) = torque_i;

        qdd_response = state_response(q, qd, torque_i, inertia_matrix, coriolis_matrix, gravity);

        qd_response = vel_response.integral(qdd_response);
        q_response  = pos_response.integral(qd_response);

    }
    return torque;
}


Eigen::Vector2d InvDynamics::state_response(Eigen::Vector2d& q_des, Eigen::Vector2d& qd_des,
        Eigen::Vector2d& torque, Eigen::Matrix2d& inertia, Eigen::Matrix2d& coriolis, Eigen::Vector2d& gravity)
{
    Eigen::Vector2d qdd_computed;

    qdd_computed = inertia.inverse() * (torque - coriolis * qd_des - gravity);

    return qdd_computed;
}

Eigen::Matrix2d Dynamics::inertia_tensor(Eigen::Vector2d& I){
    Eigen::Matrix2d I_t;

    //Simplified Version - Will need to provide a more generalised function
    int n = I.size();
    while(n != I.size()){
        for(int m = 0; m != I_t.cols(); ++m){
            I_t(n, m) = I.sum();
            I(n) = 0;
        }
        ++n;
    }

    return I_t;
}


Eigen::Matrix2d Dynamics::get_inertia_matrix(Eigen::Vector2d& q)
{
    //Obtaining the Inertia Matrix using the Jacobian

    //Eigen::MatrixXd Jacobian = get_jacobian(q);
    Eigen::Matrix2d M;
    M = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished();

    for(int n = 0; n != q.size(); ++n){
        M += bot.mass(n) * get_jacobian(q, n) * get_jacobian(q, n).transpose();
    }

    M += inertia_tensor(Iq);
    return M;
}


Eigen::Matrix2d Dynamics::get_coriolis_matrix(Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    for(int i =0; i != q.size(); ++i)
    {
        0.5 * qd(i-1) +


    }


}

Eigen::Vector2d Dynamics::get_gravity(Eigen::Vector2d& q)
{
    Eigen::Vector2d grav;
    grav = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();

    for(int i= 0; i != q.size(); ++i){
        int n = 0;
        double dq = 0.0;
        while(n <= i){
            dq += q(n);
            ++n;
        }
        grav(i) = g * cos(dq);
    }
    return grav;
}


Eigen::Matrix2d Dynamics::get_jacobian(Eigen::Vector2d& q, int link)
{
    Eigen::Matrix2d J;
    //Two Rows as the manipulator end-effector has only 2DOF
    J  = Eigen::Matrix2d::Zero();


    J = Component[X](J, q, link);
    J = Component[Y](J, q, link);

    return (J);
}


Eigen::Vector2d InvDynamics::get_gravity(Eigen::Vector2d& q)
{
    Eigen::Vector2d grav;
    grav = Eigen::Vector2d::Zero();

    for(int i= 0; i != q.size(); ++i){
        for(int n = i; n != q.size(); ++n){
            int m = i;
            double dq = 0.0;
            double l = 0.0;
            while(m <= n){
                dq += q(m);
                if(m = n) l = link_cm(m);
                else l = link_length(m);
                grav(i) += bot.mass(n) * g * l * cos(dq);
                l = link_length(m);
                ++m;
            }
        }
    }
    return grav;
}


