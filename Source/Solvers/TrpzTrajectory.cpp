//
#include "TrpzTrajectory.h"

void TrapezTrajectory::prioritise()
{
    if(fabs(end.finq(0) - s.q(0)) < fabs(end.finq(1) - s.q(1))){
        hi = fabs(end.finq(1) - s.q(1));
        hj =fabs(end.finq(0) - s.q(0));
        joint = 1;
    }
    else{
        hi = fabs(end.finq(0) - s.q(0));
        hj = fabs(end.finq(1) - s.q(1));
        joint = 0;
    }
}

void TrapezTrajectory::re_prioritise()
{
    if((joint = 0))joint = 1;
    else joint = 0;
}


Eigen::MatrixX2d TrapezTrajectory::tr_traj() {
    prioritise();

    //Acc_max & Vel_max are obtained from the SBot
    //Defining the desired orientation of first angle (angle with largest rotation) the change in angle is denoted hi

    //timestep iterator
    t = 0.0;

    //Matrix containing the joint angles at each time step to return to q_traj is TrapezTrajectory::Qa

    //Change sign of acceleration for a negative rotation (clockwise)
    if (hj < 0) Amax = -1 * Amax;


    if( hi >= (pow(bot.Vmax,2)/bot.Amax)) {

        map[acc_phase] (Qa, joint, Amax);

        map[const_velocity] (Qa, joint, Amax);

        map[decel_phase] (Qa, joint, Amax);
    }
    else{
            // Ta = sqrt(hi / Amax);
            //T = 2 * Ta;
            //In this case Vmax = hi / Ta; as the motors Vmax is never reached

            map[acc_phase] (Qa, joint, Amax);

            map[decel_phase] (Qa, joint, Amax);
    }

    re_prioritise();

    t = 0.0; //reset time index

    double Aj = hj / (Ta * (T - Ta));
    // Vj = hj / (T - Ta);

    if (hj < 0) Aj = -1 * Aj;

    //resets for joint angle and time iterator
    x = 0;
    t = 0.0; //reset time index

    //Trajectory phases of joint with equal or smaller rotation required
    map[acc_phase] (Qa, joint, Aj);
    map[decel_phase] (Qa, joint, Aj);

    return Qa;
}




