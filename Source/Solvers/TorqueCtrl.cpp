
#include "TorqueCtrl.h"

//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//The solver will comprise the Dynamics and Inverse Dynamics member functions

Dynamics::Dynamics(State& state, SBot& sbot) : s(state), bot(sbot){
    Inertia  = Eigen::Matrix2d::Zero();
    Gravity  = Eigen::Matrix2d::Zero();
    Coriolis = Eigen::Matrix2d::Zero();
    Torque   = std::make_unique<Eigen::Matrix2Xd>(2, 100);

    Rq = Eigen::Rotation2Dd(s.q);
    link2_force = (Eigen::Vector2d(2, 1) << 0.0, 0.0).finished();

    //robot manipulator parameters
    Iq << bot.link_inertia(0), bot.link_inertia(1);
    //Inertia of link with respect ot base frame: I = Rq * Eigen::Vector2d(sbot.Inertia1, sbot.Inertia2) * Rq.transpose();

    l << bot.link_length(0), bot.link_length(1);
    link_cm << bot.link_cm(0), bot.link_cm(1);
    g = 9.81;

    //Jacobian Parameters
    Component[X] = [&] (Eigen::MatrixXd Jac, Eigen::VectorXd q) -> Eigen::MatrixXd
    {
        double x = 0;
        for(int i = 0; i != q.size(); ++i){
            for(int n = i; n !=q.size(); ++n){
                int m = i;
                double dq = 0.0;
                while(m <= n) {
                    dq += q(m);
                    ++m;
                }
                x = l(n)* sin(dq);
                Jac(0, i) = Jac(0, i) - x;
            }

        }
        return Jac;
    };
    Component[Y] = [&] (Eigen::MatrixXd Jac, Eigen::VectorXd q) -> Eigen::MatrixXd
    {
        double x = 0;
        for(int i = 0; i != q.size(); ++i){
            for(int n = i; n != q.size(); ++n){
                int m = i;
                double dq = 0.0;
                while(m <= n) {
                    dq += q(m);
                    ++m;
                }
                x = l(n)* cos(dq);
                Jac(1, i) = Jac(1, i) + x;
            }

        }
        return Jac;
    };
}

//Eigen::Vector2d TorqueOut(TorqueInt& Tau, Eigen::Vector2d& Dynamics){}


Eigen::Vector2d Dynamics::forward_recursion_1(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::Vector2d Ac;

    //Linear acceleration : components of acceleration in x and y with respect to link frame
    Ac << qd(0) * qd(0) * link_cm(0), - qdd(0) * link_cm(0);

    Eigen::Matrix3d Rq1_T = Rot(q(0)).transpose();
    Gravity.col(0) = Rq1_T.col(1) * bot.mass1 * g;

    return Ac;
}


//Generic recursion function for N-link manipulators

Eigen::MatrixXd Dynamics::forward_recursion(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q)
{
    Eigen::MatrixXd Ac;

    //Linear acceleration : components of acceleration in x and y with respect to link frame
    Ac.col(0) << qd(0) * qd(0) * link_cm(0), - qdd(0) * link_cm(0);


    for(int n = 1; n != q.size(); ++n)
    {
        double qcd  = 0.0;
        double qcdd = 0.0;  //sum of joint rotation components
        int m = 1;  //joint component iterator
        while( m <= n){
            qcd += qd(m);
            qcdd += qdd(m);
            ++m;
        }
        Eigen::Vector2d An_1, An;
        An_1 = (Rot(q(n-1)).transpose() * Ac.col(n-1));
        An = (Eigen::Vector2d(2, 1) << -pow(qcd, 2) * link_cm(n), qcdd * link_cm(n)).finished();
        Ac.col(n) = An_1 + An;
    }

    Eigen::Matrix3d Rq1_T = Rot(q(0)).transpose();
    Gravity.col(0) = Rq1_T.col(1) * bot.mass1 * g;

    return Ac;
}



Eigen::Vector2d Dynamics::forward_recursion_2(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q, Eigen::Vector2d& linear_acc1)
{
    Eigen::Vector2d Ac1;
    Eigen::Vector2d Ac2;
    Eigen::Matrix2d Rq2 = Rot(q(1));

    //Linear acceleration : components of acceleration in x and y with respect to link frame
    Ac1 = (Rq2.transpose() * linear_acc1);
    Ac2 << link_cm(1) * (pow(qd(0) + qd(1), 2), (qdd(0) + qdd(1)) * link_cm(1));

    double q2 = q(0) + q(1);
    Eigen::Matrix2d Rq2_T = Rot(q2).transpose();
    Gravity.col(1)= Rq2_T.col(1) * bot.mass2 * g;

    return Ac1 - Ac2;
}


double Dynamics::backward_recursion_2(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q, Eigen::Vector2d& linear_acc2)
{
    Eigen::Vector2d f2;
    f2 = bot.mass2 * linear_acc2 + Gravity.col(1);

    double tau;
    tau = Iq(0) * (qdd(0) + qdd(1)) + f2(2)* link_cm(1);

    link2_force = f2;
    return tau;
}


double Dynamics::backward_recursion_1(Eigen::Vector2d& qdd, Eigen::Vector2d& qd, Eigen::Vector2d& q, double torque2, Eigen::Vector2d& linear_acc1)
{
    Eigen::Vector2d f1;
    Eigen::Matrix2d R1_2 = Rot(q(1));

    f1 = bot.mass1 * linear_acc1 + R1_2.col(1)* link2_force(1) - Gravity.col(0);

    double tau;
    tau = torque2 - f1(1) * link_cm(0) - R1_2(1,1) * link2_force(1) * l(0) + Iq(0) * qdd(0);

    return tau;
}


Eigen::Matrix2Xd Dynamics::get_torque(Eigen::MatrixX2d& qdd_traj, Eigen::MatrixX2d& qd_traj, Eigen::MatrixX2d& q_traj)
{
    Eigen::Vector2d lin_acc1, lin_acc2;
    Eigen::Matrix2Xd torque;


    for(int i = 0; i != q_traj.rows(); ++i)
    {
        Eigen::Vector2d q, qd, qdd;
        qdd << qdd_traj(i, 0), qdd_traj(i, 1);
        qd  << qd_traj(i, 0), qd_traj(i, 1);
        q   << q_traj(i, 0), q_traj(i, 1);


        lin_acc1 = forward_recursion_1(qdd, qd, q);

        lin_acc2 = forward_recursion_2(qdd, qd, q, lin_acc1);

        torque(1, i) = backward_recursion_2(qdd, qd, q, lin_acc2);

        torque(0, i) = backward_recursion_1(qdd, qd, q, torque(1, i), lin_acc1);

    }

    //We are only concerned with the effect of gravity in the y-direction.
    Gravity = Gravity.row(2).transpose();

    return torque;
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


Eigen::MatrixXd Dynamics::get_inertia_matrix(Eigen::VectorXd& q)
{
    //Obtaining the Inertia Matrix using the Jacobian

    Eigen::MatrixXd Jacobian = get_jacobian(q);
    Eigen::Matrix2d M;

    M = bot.mass1 * Jacobian.col(0) * Jacobian.transpose().col(0) +
            bot.mass2 * Jacobian.col(1) * Jacobian.transpose().col(1) + inertia_tensor(Iq);

    return M;
}


void Dynamics::get_coriolis_matrix()
{

}

Eigen::VectorXd Dynamics::get_gravity_vector(Eigen::VectorXd& q){
    Eigen::Matrix<double, q.size(), 1> g;

    for(int i= 0; i != q.size(); ++i){
        int n = 0;
        double dq = 0.0;
        while(n <= i){
            dq += q(n);
            ++n;
        }
        g(i) = bot.mass(i) * g * cos(dq);
    }
    return g;
}


Eigen::MatrixXd Dynamics::get_jacobian(Eigen::VectorXd& q)
{
    std::unique_ptr<Eigen::MatrixXd> J;
    //Two Rows as the manipulator end-effector has only 2DOF
    J  = std::make_unique<Eigen::MatrixXd> (2, q.size());

    *J = Component[X](*J, q);

    *J = Component[Y](*J, q);

    return (*J);
}
