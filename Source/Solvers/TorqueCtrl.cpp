
#include "TorqueCtrl.h"
//TODO: Remove Location map and attached acceleration functions; fix backward recursion operand problem and adjust the get_gravity function;
//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//The solver will comprise the Dynamics and Inverse Dynamics member functions

Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    //robot manipulator parameters from SBot
    Iq          << bot.link_inertia(0), bot.link_inertia(1);
    link_length << bot.link_length(0), bot.link_length(1);
    link_cm     << bot.link_cm(0), bot.link_cm(1);
    g  = 9.81;
    dt = 0.01;

}


//Forward & Backward recursion function for Two-link manipulator
Eigen::Vector2d Dynamics::get_torque(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& qdd)
{
    Eigen::Matrix2d Ac, Ae, force;
    Eigen::Matrix2d gravity = get_gravity(q);
    Eigen::Vector2d An_1, Ac_n, Ae_n, torque;
    Ac = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at centre of mass
    Ae = (Eigen::Matrix2d(2, 2) << 0.0, 0.0, 0.0, 0.0).finished(); //linear acceleration at end of link
    int links = q.rows();
    double qd_sum, qdd_sum;

    //Linear accelerations : components of acceleration in x and y (2D case) with respect to individual link frames
    for(int n = 0; n != links; ++n)
    {
        qd_sum = 0.0; //sum of angular velocity components
        qdd_sum = 0.0;  //sum of angular acceleration components
        int m = 0;  //joint component iterator
        while (m <= n) {
            qd_sum += qd(m);
            qdd_sum += qdd(m);
            ++m;
        }
        if (m > 1) An_1 = (Rot(q(n))) * Ae.col(n - 1);
        else An_1 = (Eigen::Vector2d(2) << 0.0, 0.0).finished();
        Ac_n << -1 * pow(qd_sum, 2) * link_cm(n), qdd_sum * link_cm(n);
        Ae_n << -1 * pow(qd_sum, 2) * link_length(n),  qdd_sum * link_length(n);
        Ae.col(n) = An_1 + Ae_n + gravity.col(n);
        Ac.col(n) = An_1 + Ac_n + gravity.col(n);
    }

    Eigen::Vector2d nextlink_force;

    for(int n = (links -1); n >= 0; --n)
    {
        qdd_sum = 0.0;
        signed int m = n;
        while(m >= 0){
            qdd_sum += qdd(m);
            --m;
        }
        if( n == (links - 1)) {
            force.col(n) = bot.mass(n) * Ac.col(n);
            torque(n) = Iq(n) * qdd_sum + (force(1, n) * link_cm(n));
        }
        else {
            nextlink_force = Rot(q(n+1)).transpose() * force.col(n+1);
            force.col(n) = bot.mass(n) * Ac.col(n) + nextlink_force;
            torque(n) = torque(n + 1) + (force(1, n) * link_cm(n)) + (nextlink_force(1) * link_cm(n)) + Iq(n) * qdd_sum;
        }
    }
    return torque;
}



std::vector<Eigen::Vector2d> TorqueController::feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj, std::vector<Eigen::Vector2d>& vel_traj, std::vector<Eigen::Vector2d>& acc_traj)
{
    Integrator velocity_response(s.qd, s.qdd, dt);
    Integrator position_response(s.q, s.qd, dt);
    Eigen::Matrix2d linear_acc, inertia;
    std::vector<Eigen::Vector2d> torque;
    Eigen::Vector2d pos_response, vel_response, acc_response, torque_i;
    Eigen::Vector2d initial_position, q, qd, qdd, pos_error, vel_error, acc_error;

    q = s.q; //Initial (State) position of the links before movement
    pos_response = pos_traj[0];
    vel_response = vel_traj[0];
    acc_response = acc_traj[0];
    Eigen::Vector2d coriolis, gravity;
    //double pos_check, vel_check, acc_check, q_check, qd_check, qdd_check; //variables checking the joint rotation protperties

    for(int i = 0; i != pos_traj.size(); ++i)
    {
        qdd = acc_traj[i];
        qd  = vel_traj[i];
        q   = pos_traj[i];

        gravity  = get_gravity_vector(q);
        inertia  = get_inertia_matrix(q, gravity);
        coriolis = get_coriolis_vector(q, qd, gravity);
        inertia_array.emplace_back(inertia);
        coriolis_array.emplace_back(coriolis);
        gravity_array.emplace_back(gravity);

        pos_error = q - pos_response;
        vel_error = qd - vel_response;
        acc_error = qdd - acc_response;


        torque_i = Dynamics::get_torque(q, qd, qdd) + (Kv * vel_error) + (Kp * pos_error);
        torque.emplace_back(torque_i);

        acc_response = TorqueController::state_response(torque_i, inertia, coriolis, gravity);
        acceleration_response.emplace_back(acc_response);

        vel_response = velocity_response.integration(acc_response);
        pos_response = position_response.trapez_integration(vel_response);
        position_response_array.emplace_back(pos_response);

    }
    return torque;
}


Eigen::Vector2d TorqueController::state_response(Eigen::Vector2d& torque, Eigen::Matrix2d& inertia, Eigen::Vector2d& coriolis, Eigen::Vector2d& gravity)
{
    Eigen::Vector2d acc_computed;

    acc_computed = inertia.inverse() * (torque - coriolis - gravity);

    return acc_computed;
}


Eigen::Vector2d TorqueController::get_gravity_vector(Eigen::Vector2d& q)
{
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    Eigen::Vector2d gravity_vec = get_torque(q, at_rest, at_rest);

    return gravity_vec;
}

//Inertia component of the torques: Inertia Matrix M(q) * qdd(angular acceleration vector);
Eigen::Matrix2d TorqueController::get_inertia_matrix(Eigen::Vector2d& q, Eigen::Vector2d& gravity)
{
    Eigen::Matrix2d inertia_matrix;
    Eigen::Vector2d at_rest  = Eigen::Vector2d::Zero();
    Eigen::Vector2d const_acc = (Eigen::Vector2d(2) << 1.0, 0.0).finished();

    for(int i = 0; i != q.size(); ++i)
    {
        inertia_matrix.col(i) = get_torque(q, at_rest, const_acc) - gravity;
        if( i < (q.size() -1)) {
            const_acc(i) = 0.0;
            const_acc(i + 1) = 1.0;
        }
    }
    return inertia_matrix;
}

//Coriolis (coriolis + centrifugal) component of the torques: Coriolis Matrix C(q, qd) * qd;
Eigen::Vector2d TorqueController::get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& gravity)
{
    Eigen::Vector2d coriolis_vec;
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    coriolis_vec = get_torque(q, qd, at_rest) - gravity;

    return coriolis_vec;
}


Eigen::Matrix2d Dynamics::get_gravity(Eigen::Vector2d& q)
{
    Eigen::Matrix2d grav;
    grav = Eigen::Matrix2d::Zero();

    Eigen::Vector2d g_vec;
    g_vec << 0.0, g;

    for(int i= 0; i != 1; ++i){
        int n = 0;
        double dq = 0.0;
        while(n <= i){
            dq += q(n);
            ++n;
        }
        grav.col(i) = Rot(dq) *  g_vec;
    }
    return grav;
}

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