
#ifndef TWOLINK_MANIP_TRAPEZTRAJECTORY_H
#define TWOLINK_MANIP_TRAPEZTRAJECTORY_H

#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/State.h"
#include <Eigen/Dense>
#include "../Common/Common.h"
#include <math.h>

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
    double timestep;
    int i;
    int j;

public:
    Eigen::MatrixX2d q_traj;
    Eigen::MatrixX2d qd_traj;
    Eigen::MatrixX2d qdd_traj;


    explicit TrapezTrajectory(State& state, ExecInt& input, SBot& sbot) : s(state), end(input), bot(sbot){
        q_traj   = Eigen::MatrixX2d::Zero();
        qd_traj  = Eigen::MatrixX2d::Zero();
        qdd_traj = Eigen::MatrixX2d::Zero();
        i = 0;
        j = 1;
        hi = 0.0; //largest joint rotation
        hj = 0.0; //smallest joint rotation
        timestep = 0.01;
    };

    void prioritise();

    void tr_traj();

    void velocity_traj(){qd_traj = derivative_array(q_traj, timestep);}


    void acc_traj(){qdd_traj = derivative_array(qd_traj, timestep);}
};



#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
