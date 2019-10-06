
#include "TorqueCtrl.h"

//The TorqueCtrl(Torque Control) will evaluate the required Torque on eah joint,
//to obtain the desired End-Effector position
//This functions comprising the solver will be the Dynamics, Inverse Dynamics as shown below

//Eigen::Vector2d TorqueOut(TorqueInt& Tau, Eigen::Vector2d& Dynamics){}


Dynamics::Dynamics(State& input) : state(input){}


Eigen::Matrix2d  Dynamics::inertia_matrix()
{
    Eigen::Matrix2d M;
    return M;
}

Eigen::Matrix2d Dynamics::get_gravity()


