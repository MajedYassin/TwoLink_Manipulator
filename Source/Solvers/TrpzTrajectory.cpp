//
#include "TrpzTrajectory.h"


Eigen::MatrixX2d tr_traj(State s, ExecInt end, SBot bot, double& dt) {
    //Acc_max & Vel_max are obtained from the SBot s.variable
    //Defining the desired orientation of first angle (angle with largest rotation) this will be changed
    double hi, hj; //Temporary variables
    int i, j;
    //else qi = input.q(1);
    double joint_1_rotation = fabs(end.finq1 - s.q(0));
    hi = std::max(fabs(end.finq1 - s.q(0)), fabs(end.finq2 - s.q(1))); //largest joint rotation
    hj = std::min(fabs(end.finq1 - s.q(0)), fabs(end.finq2 - s.q(1))); //smallest joint rotation
    if(hi = joint_1_rotation) i = 0; j = 1;
    else j = 0; i = 1;

    // double qi = hi + (s.q(0));
    int *x = new int(1);
    *x = 0;
    double Ta = 0;
    double T = 0;
    double Amax = bot.Amax;
    Eigen::MatrixX2d Qa;
    double q = 0.0;
    double t= 0; //time index

    if (hi < 0) Amax = -1 * Amax;

    if (hi >= (pow(bot.Vmax, 2)) / fabs(Amax))
        Ta = bot.Vmax / fabs(Amax);
    T = ((fabs(hi)) * fabs(Amax) + (pow(bot.Vmax, 2))) / (fabs(Amax) * bot.Vmax);
    for (t = 0; t != Ta; t += dt) {
        q = s.q(i) + 0.5 * Amax * pow((t - 0.0), 2);
        Qa(*x, i) = q;
        ++(*x);
    }

    for (t = Ta; t != T - Ta; t += dt){
        q = s.q(i) + Amax * Ta * (t - Ta / 2);
        Qa(*x, i) = q;
        ++(*x);
    }

    for (t = Ta; t != T - Ta; t += dt){
        q = end.finq(i) + Amax * Ta * (t - Ta / 2);
        Qa(*x, i) = q;
        ++(*x);
    t = T-Ta : ti : T
            q = qi - 0.5*Amax*(T - t)^2;
        Qa(*x, i) = q;
        ++(*x);


    /*
    %%Trajectory Plot
    Vmax = 10.5;
    Amax = 9;
    qi = - pi;
    q0 = 0;
    hi = qi - q0;
    t0 = 0;
    ti = 0.1;
    qia = cell(25, 1);
    x = 1;

    if( qi < 0)
            Amax = -1 * Amax;
    end

    if (hi >= (Vmax^2)/abs(Amax))
        Ta = Vmax / abs(Amax);
        T  = ((abs(qi - q0))* abs(Amax) + Vmax^2)/(abs(Amax) * Vmax);
        for t = t0:ti:Ta
           q = q0 + 0.5*Amax*(t - t0)^2;
           qia{x} = q;
           x = x + 1;
        end
        for t = Ta :ti:(T - Ta)
            q = q0 + Amax*Ta * (t - Ta/2);
            qia{x} = q;
            x = x + 1;
        end
        for t = Ta : ti : T
            q = qi - 0.5*Amax*(T - t)^2;
            qia{x} = q;
            x = x + 1;
        end
    else
        Ta = sqrt(hi/Amax);
        T  = 2* Ta;
        Vmax = hi / Ta;
        for t = t0 : ti : Ta
            q = q0 + 0.5*Amax*(t - t0)^2;
            qia{x} = q;
            x = x + 1;
        end
        for t = Ta:ti:T
            q = qi - 0.5*Amax*(T - t)^2;
            qia{x} = q;
            x = x + 1;
        end
        clear q;
    end


    qj = pi /3;
    hj = qj - q0;

    Aj = hj / (Ta*(T-Ta));
    Vj = hj / (T - Ta);

    %%Second Joint

    qja = cell(25, 1);

    x = 1;
    for tj = t0:ti:Ta
        q = q0 + 0.5*Aj*(tj - t0)^2;
        qja{x} = q;
        x = x + 1;
    end
    for tj = (T - Ta):ti:T
        q = qj - 0.5*Aj*(T - tj)^2;
        qja{x} = q;
        x = x + 1;
    end

    Q = cell2mat(qia);
    Qj = cell2mat(qja);
    dQ = gradient(Q)/ti;
    dQj = gradient(Qj)/tj;

    time = 0:ti:T;

     */
}

Eigen::MatrixX2d n_traj(State& s, ExecInt& end, SBot& bot, double timestep)
{



}