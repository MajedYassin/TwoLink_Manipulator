//
#include "TrpzTrajectory.h"

//prioritising which trajectory to plot first;
//The acceleration and velocity of the shorter joint rotation will be dependent
// on the acceleration time (Ta) and duration (T) of the larger joint rotation
void TrapezTrajectory::prioritise()
{
    if(fabs(end.finq(0) - s.q(0)) < fabs(end.finq(1) - s.q(1))){
        hi =(end.finq(1) - s.q(1));
        hj =(end.finq(0) - s.q(0));
        joint = 1;
    }
    else{
        hi = (end.finq(0) - s.q(0));
        hj = (end.finq(1) - s.q(1));
        joint = 0;
    }
}

void TrapezTrajectory::re_prioritise()
{
    if((joint = 0))joint = 1;
    else joint = 0;
    joint_traj_i_done = true;
}

double TrapezTrajectory::acc_time()
{
    if (fabs(hi) >= (pow(bot.Vmax, 2) / bot.Amax)) {
        Ta = Vmax / bot.Amax;
    }
    else Ta = sqrt(hi/A);
    return Ta;
}

double TrapezTrajectory::duration()
{
    if (fabs(hi) >= (pow(bot.Vmax, 2) / bot.Amax)){
        T = hi * A + pow(Vmax, 2)/ bot.Amax * Vmax;
    }
    else T = 2 * Ta;
}

void TrapezTrajectory::joint_acceleration()
{
    if(joint_traj_i_done) {
        A = hj / (Ta * (T - Ta));
    }
    else{
        if(hi < 0) A = -1 * bot.Amax;
        else A = bot.Amax;
    }
}

Eigen::MatrixX2d TrapezTrajectory::tr_traj(){

    prioritise();

    //Amax & Vmax are obtained directly from the SBot and are the actuators maximum acceleration and velocity

    //timestep iterator
    t = 0.0;

    //Matrix containing the joint angles at each time step to return to q_traj is TrapezTrajectory::Qa

    //Change sign of acceleration for a negative rotation (clockwise)
    if (hi < 0) A = -1 * A;


    if( fabs(hi) >= (pow(bot.Vmax,2)/bot.Amax)) {

        map[acc_phase] (Qa, joint);

        map[const_velocity] (Qa, joint);

        map[decel_phase] (Qa, joint);
    }
    else{
            //Ta = sqrt(hi / A);
            //T = 2 * Ta;
            //In this case Vmax = hi / Ta; as the motors actual Vmax is never reached

            map[acc_phase] (Qa, joint);

            map[decel_phase] (Qa, joint);
    }

    re_prioritise();

    t = 0.0; //reset time index

    joint_acceleration();
    // Vj = hj / (T - Ta);

    //resets for joint angle and time iterator
    x = 0;
    t = 0.0; //reset time index

    //Trajectory phases of joint with equal or smaller rotation required
    map[acc_phase] (Qa, joint);
    map[decel_phase] (Qa, joint);

    return Qa;
}
