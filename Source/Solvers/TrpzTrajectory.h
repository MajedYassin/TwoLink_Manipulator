
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
        q_traj   = tr_traj();
        qd_traj  = velocity_traj();
        qdd_traj = acc_traj();
        i = 0; //first joint to be evaluated
        j = 1;
        hi = 0.0; //larger rotation
        hj = 0.0; //smaller rotation
        timestep = 0.01;
    };

    void prioritise();

    Eigen::MatrixX2d adjust_traj(Eigen::MatrixX2d& Q);

    Eigen::MatrixX2d tr_traj();

    Eigen::MatrixX2d velocity_traj(){return derivative_array(q_traj, timestep);}


    Eigen::MatrixX2d acc_traj(){return derivative_array(qd_traj, timestep);}
};



#endif //TWOLINK_MANIP_TRAPEZTRAJECTORY_H
