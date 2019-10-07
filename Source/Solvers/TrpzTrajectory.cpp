//
#include "TrpzTrajectory.h"


Eigen::MatrixX2d tr_traj(State s, ExecInt end, SBot bot, double& dt)
{
    //Acc_max & Vel_max are obtained from the SBot s.variable
    //Defining the desired orientation of first angle (angle with largest rotation) this will be changed
    double qi, qj; //Temporary variables
    //if(end.q(0) >= end.q(1)) qi = end.q(0);
    //else qi = input.q(1);


    double hi = qi - (s.q(0));


    Eigen::Vector2d q;
    q << 8, 9, 10;

    return q;

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