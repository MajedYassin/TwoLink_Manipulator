
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/State.h"
#include <Eigen/Dense>

struct TrapezTrajectory{
private:
    State& s;
    ExecInt& end;
    SBot& bot;
    double hi;
    double hj;
    double timestep;

public:
    Eigen::VectorXd q_traj;


    explicit TrapezTrajectory(State& state, ExecInt& input, SBot& sbot) : s(state), end(input), bot(sbot) {
        q_traj = Eigen::VectorXd::Zero();
        hi = std::max(end.finq1 - s.q(0), end.finq2 - s.q(1)); //largest joint rotation
        hj = std::min(end.finq1 - s.q(0), end.finq2 - s.q(1)); //smallest joint rotation
        timestep = 0.01;

    };

    void set_traj(){
        if( hi >= (pow(bot.Vmax,2)/bot.Amax)) {
            q_traj = tr_traj(s, end, bot, timestep);
        }
        else {
            q_traj = traj();
        }
    }


    void velocity_traj(){
        return derivative_array(q_traj, timestep);
    }

};

Eigen::VectorXd tr_traj(State& s, SBot& bot);

#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
