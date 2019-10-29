//
#include "TrpzTrajectory.h"

TrapezTrajectory::TrapezTrajectory(SBot& sbot, State& state) : bot(sbot), s(state){
        t = 0.0;
        h << Eigen::Vector2d::Zero();
        hmax = 0.0; //Longest joint rotation requested
        A = Eigen::Vector2d::Zero(); //Joint Actuator acceleration
        Vmax = bot.Vmax;
        dt = 0.01; //timestep
        T[a] = 0.0; // Acceleration time
        T[d] = 0.0; // Duration of Joint rotation
        x = 0; //iterator

        //TODO: - Replace Eigen for pass by reference in map[]; -Add note to Torque Dynamics to explain order of calling the traj functions: q_traj, qd_traj, qdd_traj;

        phase[acc_phase] = [&](std::vector<Eigen::Vector2d>& Q, Eigen::Vector2d& A, std::map<const Time, double> T, Eigen::Vector2d& q0){
            Eigen::Vector2d qi;
            while (t != T[a]){
                for (int n = 0; n != A.size(); ++n){
                    qi(n) = q0(n) + (0.5 * A(n)) * pow((t - 0.0), 2);
                }
                Q.emplace_back(qi);
                t += dt;
            }
            return Q;
        };
        phase[const_velocity] = [&] (std::vector<Eigen::Vector2d>& Q, Eigen::Vector2d& A, std::map<const Time, double> T, Eigen::Vector2d& q0){
            Eigen::Vector2d qi;
            t = T[a];
            while (t != T[d] - T[a]) {
                for (int n = 0; n != A.size(); ++n) {
                    qi(n) =q0(n) + A(n) * T[a] * (t - T[a] / 2);
                }
                Q.emplace_back(qi);
                t += dt;
            }
            return Q;
        };
        phase[decel_phase] = [&] (std::vector<Eigen::Vector2d>& Q, Eigen::Vector2d& A, std::map<const Time, double> T, Eigen::Vector2d& qf){
            Eigen::Vector2d qi;
            t = T[d] - T[a];
            while (t != T[d]) {
                for (int n = 0; n != A.size(); ++n) {
                    qi(n) = qf(n) - A(n) * T[a] * pow((T[d] - t), 2);
                }
                Q.emplace_back(qi);
                t += dt;
            }
            return Q;
        };
};


std::vector<Eigen::Vector2d> TrapezTrajectory::pos_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf){
    q_traj = tr_traj(q0, qf);
    return q_traj;
}


std::vector<Eigen::Vector2d> TrapezTrajectory::vel_traj(){
    qd_traj = derivative_array(q_traj, dt);
    return qd_traj;
}

//Need fix to allow user to request only acc_traj without calling velocity_traj
std::vector<Eigen::Vector2d> TrapezTrajectory::acc_traj(){
    return derivative_array(qd_traj, dt);
}


//prioritising which trajectory to plot first;
//The acceleration and velocity of the shorter joint rotation will be dependent
// on the acceleration time (Ta) and duration (T) of the larger joint rotation
void TrapezTrajectory::prioritise()
{
    for(int n = 0; n != h.size(); ++n) hmax = std::max(hmax, fabs(h(n)));
}


void TrapezTrajectory::acc_time()
{
    if (fabs(hmax) >= (pow(bot.Vmax, 2) / bot.Amax)) {
        T[a] = Vmax / bot.Amax;
    }
    else T[a] = sqrt(hmax/bot.Amax);
}


void TrapezTrajectory::duration()
{
    if (fabs(hmax) >= (pow(bot.Vmax, 2) / bot.Amax)){
        T[d] = hmax * bot.Amax + pow(Vmax, 2)/ bot.Amax * Vmax;
    }
    else T[d] = 2 * T[a];
}


void TrapezTrajectory::joint_acceleration()
{
    for(int n = 0; n != A.size(); ++n){
        if((h(n) = hmax)) A(n) = (h(n)/fabs(h(n))) * bot.Amax;//h(n)/fabs(h(n)) verifies if negative rotation (clockwise)
        else A(n) = h(n) / (T[a] * (T[d] - T[a]));
    }
}

std::vector<Eigen::Vector2d> TrapezTrajectory::tr_traj(Eigen::Vector2d& q0, Eigen::Vector2d& qf){

    //Change in joint orientation q0 = initial joint angles, qf = final joint angles;
    h = qf - q0;
    std::vector<Eigen::Vector2d> Qa;

    prioritise();

    //Matrix containing the joint angles at each time step to return to q_traj is TrapezTrajectory::Qa
    //Amax & Vmax are obtained directly from the SBot and are the actuators maximum acceleration and velocity

    //Calculating acceleration time and total duration time;
    acc_time();
    duration();

    //Calculating joint accelerations
    joint_acceleration();

    Qa = phase[acc_phase](Qa, A, T, q0);

    if (hmax >= (pow(bot.Vmax, 2) / bot.Amax)) {
        Qa = phase[const_velocity](Qa, A, T, q0);
    }

    Qa = phase[decel_phase] (Qa, A, T, qf);

    return Qa;
}
