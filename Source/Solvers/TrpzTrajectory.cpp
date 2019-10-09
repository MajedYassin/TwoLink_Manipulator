//
#include "TrpzTrajectory.h"

void TrapezTrajectory::prioritise()
{
    if(fabs(end.finq(0) - s.q(0)) < fabs(end.finq(1) - s.q(1))){
        hi = fabs(end.finq(1) - s.q(1));
        hj =fabs(end.finq(0) - s.q(0));
        i = 1;
        j = 0;
    }
    else{
        hi = fabs(end.finq(0) - s.q(0));
        hj = fabs(end.finq(1) - s.q(1));
        i = 0;
        j = 1;
    }
}





Eigen::MatrixX2d TrapezTrajectory::adjust_traj(Eigen::MatrixX2d& Qd){
    if ((i = 0)){
        return Qd;
    }
    else{
        Eigen::MatrixX2d Q;
        //Q << Qd.cols(1), Qd.cols(0);
        for (int n = 0; n != Qd.rows(); ++n) {
                Q(n, 0) = Qd(n, j);
                Q(n, 1) = Qd(n, i);
        }
        return Q;
    }
}


Eigen::MatrixX2d TrapezTrajectory::tr_traj() {
    prioritise();

    //Acc_max & Vel_max are obtained from the SBot s.variable
    //Defining the desired orientation of first angle (angle with largest rotation) this will be changed
    double dt = timestep;

    //else qi = input.q(1);
    // double qi = hi + (s.q(0));

    int x = 0;
    double Ta = 0;
    double T = 0;
    double Amax = bot.Amax;
    double q = 0.0;
    double t0 = 0;
    double t; //time iterator

    //Matrix containing the joint angles at each time step to return to q_traj in the TrapezTrajectory object

    if( hi >= (pow(bot.Vmax,2)/bot.Amax)) {
        auto Qa = [&]() -> Eigen::Matrix2d {
            Eigen::MatrixX2d Qa;

            //Changes sign of the acceleration for
            if (hi < 0) Amax = -1 * Amax;

            if (hi >= (pow(bot.Vmax, 2)) / fabs(Amax))
                Ta = bot.Vmax / fabs(Amax);
            T = ((fabs(hi)) * fabs(Amax) + (pow(bot.Vmax, 2))) / (fabs(Amax) * bot.Vmax);
            for (t = 0; t != Ta; t += dt) {
                q = s.q(i) + 0.5 * Amax * pow((t - 0.0), 2);
                Qa(x, i) = q;
                ++x;
            }

            for (t = Ta; t != T - Ta; t += dt) {
                q = s.q(i) + Amax * Ta * (t - Ta / 2);
                Qa(x, i) = q;
                ++x;
            }

            for (t = T - Ta; t != T - Ta; t += dt) {
                q = end.finq(i) + Amax * Ta * pow((T - t), 2);
                Qa(x, i) = q;
                ++x;
            }

            return Qa;
        };
    }
    else{
        auto Qa = [&]() -> Eigen::MatrixX2d {
            Eigen::MatrixX2d Qa;

            Ta = sqrt(hi / Amax);
            T = 2 * Ta;
            //In this case Vmax = hi / Ta; as the motors Vmax is never reached

            for (t = t0; t != Ta; t += dt) {
                q = s.q(i) + 0.5 * Amax * pow(t - t0, 2);
                Qa(x) = q;
                ++x;
            }

            for (t = Ta; t != T; t += dt) {
                q = end.finq(i) - 0.5 * Amax * pow(T - t, 2);
                Qa(x) = q;
                ++x;
            }
            return Qa;
        };
    }

    auto Q = [&](Eigen::MatrixX2d& Qa) -> Eigen::MatrixX2d {
        double t = 0; //time index

        double Aj = hj / (Ta * (T - Ta));
        // Vj = hj / (T - Ta);

        if (hj < 0) Aj = -1 * Aj;

        //resets for joint angle iterator and initial joint position
        x = 0;
        q = s.q(j);

        for (t = t0; t != Ta; t += dt) {
            q = s.q(j) + 0.5 * Aj * pow(t - t0, 2);
            Qa(x, j) = q;
            ++x;
        }
        for (t = Ta; t != T + 0.001; t += dt) {
            q = end.finq(j) - 0.5 * Aj * pow(T - t, 2);
            Qa(x, j) = q;
            ++x;
        }
        return Qa;
    };
    Q = adjust_traj(Q);
    return Q;
}


auto acc_phase = [&](Eigen::MatrixX2d Q) -> Eigen::MatrixX2d {
    for (t = 0; t != Ta; t += dt) {
    q = s.q(i) + 0.5 * Amax * pow((t - 0.0), 2);
    Qa(x, i) = q;
    ++(x);
    }
}


auto const_velocity = [&](Eigen::MatrixX2d Q) -> Eigen::MatrixX2d {
    for (t = Ta; t != T - Ta; t += dt) {
        q = s.q(i) + Amax * Ta * (t - Ta / 2);
        Qa(x, i) = q;
        ++(x);
    }
}


auto deceleration_phase = [&](Eigen::MatrixX2d& Q) -> Eigen::MatrixX2d {
    for (t = T - Ta; t != T - Ta; t += dt) {
        q = end.finq(i) + Amax * Ta * pow((T - t), 2);
        Qa(x, i) = q;
        ++(x);
    }
}

auto
for (t = t0; t != Ta; t += dt) {
q = s.q(i) + 0.5 * Amax * pow(t - t0, 2);
Qa(x) = q;
++x;
}

for (t = Ta; t != T; t += dt) {
q = end.finq(i) - 0.5 * Amax * pow(T - t, 2);
Qa(x) = q;
++x;
}
retur
auto  = [&](Eigen::MatrixX2d& Qa) -> Eigen::MatrixX2d {}


auto  = [&](Eigen::MatrixX2d& Qa) -> Eigen::MatrixX2d {}



auto  = [&](Eigen::MatrixX2d& Qa) -> Eigen::MatrixX2d {}


auto  = [&](Eigen::MatrixX2d& Qa) -> Eigen::MatrixX2d {}





