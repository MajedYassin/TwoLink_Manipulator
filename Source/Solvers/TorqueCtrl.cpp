
#include "TorqueCtrl.h"
#include <cmath>


Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    //robot manipulator parameters from SBot
    Iq          << bot.link_inertia(0), bot.link_inertia(1);
    link_length << bot.link_length(0), bot.link_length(1);
    link_cm     << bot.link_cm(0), bot.link_cm(1);
<<<<<<< Updated upstream
    friction_coefficient = -0.5;
=======
    friction_coefficient = -0.2;
>>>>>>> Stashed changes
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
        else An_1 = gravity.col(n);
        Ac_n << -1 * pow(qd_sum, 2) * link_cm(n), qdd_sum * link_cm(n);
        Ae_n << -1 * pow(qd_sum, 2) * link_length(n),  qdd_sum * link_length(n);
        Ae.col(n) = An_1 + Ae_n;
        Ac.col(n) = An_1 + Ac_n;
    }
    accelerations.emplace_back(Ac.row(1));
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
            torque(n) = torque(n + 1) + (force(1, n) * link_cm(n)) +
                        (nextlink_force(1) * link_cm(n)) + Iq(n) * qdd_sum;
        }
    }
    return torque;
}



std::vector<Eigen::Vector2d> TorqueController::feedforward_torque(std::vector<Eigen::Vector2d>& pos_traj,
            std::vector<Eigen::Vector2d>& vel_traj, std::vector<Eigen::Vector2d>& acc_traj)
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
    Eigen::Vector2d coriolis, gravity, friction;
<<<<<<< Updated upstream
=======
    //double pos_check, vel_check, acc_check, q_check, qd_check, qdd_check; //variables checking the joint rotation properties
>>>>>>> Stashed changes

    for(int i = 0; i != pos_traj.size(); ++i)
    {
        qdd = acc_traj[i];
        qd  = vel_traj[i];
        q   = pos_traj[i];

        gravity  = get_gravity_vector(q);
        inertia  = get_inertia_matrix(q, gravity);
        coriolis = get_coriolis_vector(q, qd, gravity);
        friction = get_friction(qd);


        pos_error = q - pos_response;
        vel_error = qd - vel_response;
        acc_error = qdd - acc_response;


        torque_i = Dynamics::get_torque(q, qd, qdd) + friction + (Kv * vel_error) + (Kp * pos_error);
        torque.emplace_back(torque_i);

        acc_response = TorqueController::forward_dynamics(torque_i, inertia, coriolis, gravity, friction);


        vel_response = velocity_response.integration(acc_response);
        pos_response = position_response.trapez_integration(vel_response);
        //position_response_array.emplace_back(pos_response);

    }
    return torque;
}


<<<<<<< Updated upstream
Eigen::Vector2d TorqueController::forward_dynamics(Eigen::Vector2d& torque, Eigen::Matrix2d& inertia,
            Eigen::Vector2d& coriolis, Eigen::Vector2d& gravity, Eigen::Vector2d& friction)
=======
Eigen::Vector2d TorqueController::forward_dynamics(Eigen::Vector2d& torque, Eigen::Matrix2d& inertia, Eigen::Vector2d& coriolis, Eigen::Vector2d& gravity, Eigen::Vector2d& friction)
>>>>>>> Stashed changes
{
    Eigen::Vector2d acc_computed;

    acc_computed = inertia.inverse() * (torque - coriolis - gravity);

    return acc_computed;
}


Eigen::Vector2d Dynamics::get_gravity_vector(Eigen::Vector2d& q)
{
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    Eigen::Vector2d gravity_vec = get_torque(q, at_rest, at_rest);

    return gravity_vec;
}

//Inertia component of the torques: Inertia Matrix M(q)
Eigen::Matrix2d Dynamics::get_inertia_matrix(Eigen::Vector2d& q, Eigen::Vector2d& gravity)
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
Eigen::Vector2d Dynamics::get_coriolis_vector(Eigen::Vector2d& q, Eigen::Vector2d& qd, Eigen::Vector2d& gravity)
{
    Eigen::Vector2d coriolis_vec;
    Eigen::Vector2d at_rest = Eigen::Vector2d::Zero();

    coriolis_vec = get_torque(q, qd, at_rest) - gravity;

    return coriolis_vec;
}


Eigen::Vector2d Dynamics::get_friction(Eigen::Vector2d& qd)
{
    Eigen::Vector2d friction_vec = Eigen::Vector2d::Zero();
    int n;

    for(int i = 0; i != qd.size(); ++i)
    {
        n = 0;
        while(n <= i) {
            friction_vec(i) += qd(n);
            ++n;
        }
    }

    return friction_coefficient * friction_vec;
}



Eigen::Matrix2d Dynamics::get_gravity(Eigen::Vector2d& q)
{
    Eigen::Matrix2d grav;
    grav = Eigen::Matrix2d::Zero();

    Eigen::Vector2d g_vec;
    g_vec << 0.0, g;

    for(int i= 0; i != q.size(); ++i){
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



std::vector<Eigen::Vector2d> PendulumModel::twolink_pendulum()
{
    std::vector<Eigen::Vector2d> position, velocity, acceleration;
<<<<<<< Updated upstream
    Eigen::Vector2d  coriolis, gravity, friction, torque, acc_response, vel_response, at_rest;
=======
    Eigen::Vector2d  coriolis, gravity, friction, torque, acc_response, vel_response, at_rest, constant;
    constant << -0.10, -0.10;
>>>>>>> Stashed changes
    at_rest << 0.0, 0.0;
    Eigen::Matrix2d inertia;
    position.emplace_back(s.q);
    velocity.emplace_back(at_rest);
    acceleration.emplace_back(at_rest);
    //acceleration.emplace_back(-1 * get_gravity_vec(s.q));
    //velocity.emplace_back(acceleration.back() * dt);


    Integrator velocity_response(velocity.back(), acceleration.back(), dt);
    Integrator position_response(position.back(), velocity.back(), dt);

    bool oscillating = true;
    int i = 0.0;

    while(oscillating) {

<<<<<<< Updated upstream
        gravity  = get_gravity_vector(position.back());
        inertia  = get_inertia_matrix(position.back(), gravity);
=======
        gravity = get_gravity_vector(position.back());
        inertia = get_inertia_matrix(position.back(), gravity);
>>>>>>> Stashed changes
        coriolis = get_coriolis_vector(position.back(), velocity.back(), gravity);
        friction = get_friction(velocity.back());
        gravity_array.emplace_back(gravity);

<<<<<<< Updated upstream
        torque = - gravity + friction;

        acc_response = inertia.inverse() * ( - gravity + friction - coriolis);

=======
        torque = -gravity + (inertia * acceleration.back()) + coriolis + friction;


        //torque = get_torque(position.back(), velocity.back(), acceleration.back()) + friction;
        torque_array.emplace_back(torque);

        if(i == 0) acc_response = inertia.inverse() * (torque);
        else acc_response = inertia.inverse() * (torque + gravity - coriolis);

>>>>>>> Stashed changes
        acceleration.emplace_back(acc_response);

        velocity.emplace_back(velocity_response.integration(acc_response));
        position.emplace_back(position_response.trapez_integration(velocity.back()));

<<<<<<< Updated upstream
        if(i > 50) {
            if(fabs(velocity.back()[1]) < 0.01 && fabs(velocity.back()[0]) < 0.01) oscillating = false;
            //oscillating = (i != 3000);
        }
        ++i;
    }

=======
        //oscillating  = (velocity.back()(1) != 0.0);
        oscillating = (i != 5000);
        ++i;
    }
>>>>>>> Stashed changes
    position_array = position;
    velocity_array = velocity;
    acceleration_array = acceleration;

    return position;
}


<<<<<<< Updated upstream
/*
=======

>>>>>>> Stashed changes
Eigen::Vector2d TorqueController::get_gravity_vec(Eigen::Vector2d& q)
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
                grav(i) += g * l * cos(dq);
                ++m;
            }
        }
    }
    return grav;
}
