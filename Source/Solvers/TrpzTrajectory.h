
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/State.h"
#include <Eigen/Dense>
#include "../Common/Common.h"
#include <math.h>
#include <map>
#include <functional>


Eigen::MatrixX2d tr_traj(State& s, ExecInt& end, SBot& bot, double& timestep);

//Trajectory function with no constant velocity phase (Non trapezoidal velocity profile)
Eigen::MatrixX2d n_traj(State& s, ExecInt& end, SBot& bot, double timestep);


struct TrapezTrajectory{
private:
    State& s;
    ExecInt& end;
    SBot& bot;
    double hi;
    double hj;
    int joint //joint number
    double Amax;
    double Vmax;
    double t0;
    double dt;
    double q;
    double t;
    double Ta;
    double T;
    int x; //iterator
    enum Phase {acc_phase, const_velocity, decel_phase};
    std::map<const Phase, std::function<Eigen::MatrixX2d (Eigen::MatrixX2d, double, double)>> map;
    Eigen::MatrixX2d Q;

public:
    Eigen::MatrixX2d q_traj;
    Eigen::MatrixX2d qd_traj;
    Eigen::MatrixX2d qdd_traj;


    explicit TrapezTrajectory(State& state, ExecInt& input, SBot& sbot) : s(state), end(input), bot(sbot){
        q_traj   = tr_traj();
        qd_traj  = velocity_traj();
        qdd_traj = acc_traj();
        joint = 0;
        hi = 0.0; //larger rotation
        hj = 0.0; //smaller rotation
        dt = 0.01; //timestep
        Amax = sbot.Amax;
        Vmax = sbot.Vmax;
        t0 = 0.0;
        dt = 0.01; //timestep
        q = 0.0; //joint position iterator
        t = 0.0; // time iterator
        Ta = acc_time();
        T = duration();
        int x; //iterator

        map[acc_phase] = [&](Eigen::MatrixX2d Q, int joint, double A) -> Eigen::MatrixX2d {
            for(t = 0; t != Ta; t += dt) { q = s.q(joint) + 0.5 * A * pow((t - 0.0), 2); Qa(x, joint) = q; ++(x);}};
        map[const_velocity] = [&] (Eigen::MatrixX2d Q, double q0, double qf, double A) -> Eigen::MatrixX2d {
            for(t = Ta; t != T - Ta; t += dt) {q = s.q(joint) + A * Ta * (t - Ta / 2); Qa(x, joint) = q; ++(x);}};
        map[decel_phase] = [&] (Eigen::MatrixX2d Q, double q0, double qf, double A) -> Eigen::MatrixX2d {
            for(t = T - Ta; t != T; t += dt){q = end.finq(joint) - A * Ta * pow((T-t), 2); Qa(x, joint) = q; ++(x);}};
    };

    double acc_time();


    double duration();


    void


    void prioritise();

    Eigen::MatrixX2d adjust_traj(Eigen::MatrixX2d& Q);

    Eigen::MatrixX2d tr_traj();

    Eigen::MatrixX2d velocity_traj(){return derivative_array(q_traj, timestep);}


    Eigen::MatrixX2d acc_traj(){return derivative_array(qd_traj, timestep);}
};




#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
