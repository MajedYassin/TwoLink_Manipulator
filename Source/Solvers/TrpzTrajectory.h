
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/State.h"
#include <Eigen/Dense>
#include "../Common/Common.h"

Eigen::MatrixX2d tr_traj(State& s, ExecInt& end, SBot& bot, double timestep);

//Trajectory function with no constant velocity phase (Non trapezoidal velocity profile)
Eigen::MatrixX2d n_traj(State& s, ExecInt& end, SBot& bot, double timestep);


struct TrapezTrajectory{
private:
    State& s;
    ExecInt& end;
    SBot& bot;
    double hi;
    double hj;
    double timestep;

public:
    Eigen::MatrixX2d q_traj;
    Eigen::MatrixX2d qd_traj;
    Eigen::MatrixX2d qdd_traj;


    explicit TrapezTrajectory(State& state, ExecInt& input, SBot& sbot) : s(state), end(input), bot(sbot) {
        q_traj   = Eigen::MatrixX2d::Zero();
        qd_traj  = Eigen::MatrixX2d::Zero();
        qdd_traj = Eigen::MatrixX2d::Zero();
        hi = std::max(end.finq1 - s.q(0), end.finq2 - s.q(1)); //largest joint rotation
        hj = std::min(end.finq1 - s.q(0), end.finq2 - s.q(1)); //smallest joint rotation
        timestep = 0.01;
    };

    void set_traj() {
        if( hi >= (pow(bot.Vmax,2)/bot.Amax)) {
            q_traj = tr_traj(s, end, bot, timestep);
        }
        else {
            q_traj = traj();
        }
    }



    void velocity_traj(){qd_traj = derivative_array(q_traj, timestep);}

    void acc_traj(){qdd_traj = derivative_array(qd_traj, timestep);}

};


#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
