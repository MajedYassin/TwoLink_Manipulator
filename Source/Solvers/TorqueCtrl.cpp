
#include "TorqueCtrl.h"

//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//The solver will comprise the Dynamics and Inverse Dynamics member functions

Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    //robot manipulator parameters from SBot
    Iq << bot.link_inertia(0), bot.link_inertia(1);
    link_length << bot.link_length(0), bot.link_length(1);
    link_cm << bot.link_cm(0), bot.link_cm(1);
    g  = 9.81;
    dt = 0.01;

    //Jacobian Parameters
    Component[X] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double jac_x;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double q_sum = 0.0;
                while (m <= n) q_sum += q(m); ++m;
                if ((m = l)) jac_x = link_cm(m) * sin(q_sum);
                else jac_x = link_length(m) * sin(q_sum);
                Jac(0, i) -= jac_x;
            }
        }
        return Jac;
    };
    Component[Y] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double jac_y = 0;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double q_sum = 0.0;
                while(m <= n) {
                    q_sum += q(m);
                    ++m;
                }
                if ((m = l)) jac_y = link_cm(n) * cos(q_sum);
                else jac_y = link_length(n) * cos(q_sum);
                Jac(1, i) -= jac_y;
            }

        }
        return Jac;
    };
}


//Forward recursion function for N-link manipulator
Eigen::Matrix2d Dynamics::forward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::Matrix2d Ac, Ae;
    Ac = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at centre of mass
    Ae = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at end of link

    //Linear accelerations : components of acceleration in x and y (2D case) with respect to individual link frames
    for(int n = 0; n != q.size(); ++n) {
        double qd_sum = 0.0; //sum of angular velocity components
        double qdd_sum = 0.0;  //sum of angular acceleration components
        int m = 0;  //joint component iterator
        while (m <= n) {
            qd_sum += qd(m);
            qdd_sum += qdd(m);
            ++m;
        }
        Eigen::Vector2d An_1, Ac_n, Ae_n;
        if (m > 0) An_1 = (Rot(q(m))).transpose() * Ae.col(m - 1);
        else An_1 = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();
        Ac_n = (Eigen::Vector2d(2, 1) << -1 * pow(qd_sum, 2) * link_cm(n), qdd_sum * link_cm(n)).finished();
        Ae_n = (Eigen::Vector2d(2, 1) << -1 * pow(qd_sum, 2) * link_cm(n), qdd_sum * link_length(n)).finished();
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
        double qdd_sum = 0.0;
        signed int m = n;
        while(m >= 0){
            qdd_sum += qdd(m);
            --m;
        }
        if((m < 0)) {
            force(n) = bot.mass(n) * linear_acc(2, n) + gravity(n);
            torque(n) = Iq(n) * qdd_sum + force(n) * link_cm(n);
        }
        else {
            force(n) = bot.mass(n) * linear_acc(2, n) + Rot(q(n))(1, 1) * force(n + 1) - gravity(n);
            torque(n) = torque(n + 1) - force(n) * link_cm(n) - Rot(q(n))(1, 1) * force(n + 1) * link_length(n) + Iq(n) * qdd(n);
        }
    }
    return torque;
}


Eigen::Matrix2Xd InvDynamics::feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj, std::vector<Eigen::Vector2d>& vel_traj, std::vector<Eigen::Vector2d>& acc_traj)
{
    Integrator velocity_response(s.qd, dt);
    Integrator position_response(s.q, dt);
    Eigen::Matrix2d linear_acc;
    Eigen::Matrix2Xd torque;
    Eigen::Vector2d pos_response, vel_response, acc_response, torque_i;
    Eigen::Vector2d initial_position, q, qd, qdd, pos_error, vel_error, acc_error;

    q = s.q; //Initial (State) position of the links before movement
    pos_response   = Eigen::Vector2d::Zero();
    vel_response  = Eigen::Vector2d::Zero();
    acc_response = Eigen::Vector2d::Zero();
    Eigen::Vector2d inertia, coriolis, gravity;

    for(int i = 0; i != pos_traj.size(); ++i)
    {
        initial_position = q;

        qdd = acc_traj[i];
        qd  = vel_traj[i];
        q   = pos_traj[i];

        //coriolis = get_coriolis_matrix(q, initial_position, qd, inertia);
        //inertia  = get_inertia_matrix(q);
        //gravity  = get_gravity(q);

        gravity = get_gravity_vector(q);
        inertia = get_inertia_vector(qdd, q, gravity);
        coriolis = get_coriolis_vector(qdd, qd, q, gravity, inertia);


        pos_error = q   - pos_response;
        vel_error = qd  - vel_response;
        acc_error = qdd - acc_response;

        linear_acc = forward_recursion(qdd, qd, q);
        torque_i = backward_recursion( qdd, qd, q, linear_acc) + Kv * vel_error + Kp * pos_error;

        torque.col(i) = torque_i;

        acc_response = state_response(torque_i, inertia, coriolis, gravity);

        vel_response = velocity_response.integral(acc_response);
        pos_response  = position_response.integral(vel_response);

    }
    return torque;
}


Eigen::Vector2d InvDynamics::state_response(Eigen::Vector2d& torque, Eigen::Vector2d& inertia,Eigen::Vector2d& coriolis, Eigen::Vector2d& gravity)
{
    Eigen::Vector2d acc_computed;

    acc_computed = inertia.inverse() * (torque - coriolis - gravity);

    return acc_computed;
}


Eigen::Vector2d InvDynamics::get_gravity_vector(Eigen::Vector2d& q)
{
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    Eigen::Matrix2d linear_acc = forward_recursion(at_rest, at_rest, q);

    Eigen::Vector2d gravity_vec = backward_recursion(at_rest, at_rest, q, linear_acc);

    return gravity_vec;
}

//Inertia component of the torques: Inertia Matrix M(q) * qdd(angular acceleration vector);
Eigen::Vector2d InvDynamics::get_inertia_vector(Eigen::Vector2d& q, Eigen::Vector2d& qdd, Eigen::Vector2d gravity)
{
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    Eigen::Matrix2d linear_acc = forward_recursion(qdd, at_rest, q);

    Eigen::Vector2d inertia_vec = backward_recursion(qdd, at_rest, q, linear_acc) - gravity;

    return inertia_vec;
}

//Coriolis (coriolis + centrifugal) component of the torques: Coriolis Matrix C(q, qd) * qd;
Eigen::Vector2d InvDynamics::get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& qdd, Eigen::Vector2d& gravity, Eigen::Vector2d& inertia)
{
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    Eigen::Matrix2d linear_acc = forward_recursion(at_rest, qd, q);

    Eigen::Vector2d coriolis_vec = backward_recursion(at_rest, qd, q, linear_acc) - gravity;

    return coriolis_vec;
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


//Unused Functions to independently calculate the Inertia, Coriolis and the Jacobian matrices and the gravity Vector;

/*
Eigen::Vector2d InvDynamics::get_gravity(Eigen::Vector2d& q)
{
    Eigen::Vector2d grav;
    grav = Eigen::Vector2d::Zero();
    double l;

    for(int i= 0; i != q.size(); ++i){
        for(int n = i; n != q.size(); ++n){
            int m = i;
            double dq = 0.0;
            while(m <= n){
                dq += q(m);
                if((m = n)) l = link_cm(m);
                else l = link_length(m);
                grav(i) += bot.mass(n) * g * l * cos(dq);
                ++m;
            }
        }
    }
    return grav;
}

Eigen::Matrix2d InvDynamics::inertia_tensor(Eigen::Vector2d& I){
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


Eigen::Matrix2d InvDynamics::get_jacobian(Eigen::Vector2d& q, int link)
{
    Eigen::Matrix2d J;
    //Two Rows as the manipulator end-effector has only 2DOF
    J  = Eigen::Matrix2d::Zero();

    J = Component[X](J, q, link);
    J = Component[Y](J, q, link);

    return (J);
}


Eigen::Matrix2d InvDynamics::get_inertia_matrix(Eigen::Vector2d& q)
{
    //Obtaining the Inertia Matrix using the Jacobian
    Eigen::Matrix2d M;
    M = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished();

    for(int n = 0; n != q.size(); ++n){
        M += bot.mass(n) * get_jacobian(q, n) * get_jacobian(q, n).transpose();
    }

    M += inertia_tensor(Iq);
    return M;
}


//The rotation (angular velocity) of a link results in a Coriolis force being exerted on the previous link, which is supporting it.
Eigen::Matrix2d InvDynamics::get_coriolis_matrix(Eigen::Vector2d& q, Eigen::Vector2d& q0, Eigen::Vector2d& qd, Eigen::Matrix2d& M)
{
    Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
    Eigen::Vector2d diff_M; //derivative of the Inertia Matrix w.r.t time; elements mdij = Sum of (dmijk * qdk) where k = 2;
    std::vector<Eigen::Matrix2d> dM; //difference in Inertia Matrix(q - q0) over the dq(q-q0)

    dM.clear();

    for(int i = 0; i != M.size(); ++i){
        for(int k = 0; k != q.size(); ++k) {
            dM[k] = (get_inertia_matrix(q) - M) / (q(k) - q0(k));
            C(i) += (dM[k])(i) * qd(k);
        }
    }

    for(int k = 0; k != dM.size(); ++k){
        diff_M(k) = qd.transpose() * dM[k] * qd;
    }

    C << (C * qd), - (0.5 * diff_M);
    return C;
}
*/