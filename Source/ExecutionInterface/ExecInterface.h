
#ifndef TWOLINK_MANIP_EXECINTERFACE_H
#define TWOLINK_MANIP_EXECINTERFACE_H

#include <Eigen/Dense>
#include "../Common/Common.h"
#include "../Bot/Sbot.h"

//The ExecutionInterface computes the Inital and final properties of the model e.g. pose, and
//calls the functions of the Solver to execute the calculations necessary to compute:
//The Inverse Kinematics, Forward Kinematics or Required Torque of the Two Link Robot
//The Interface runs based on parameters specified in the bot parameters struct, SBot.



struct ExecInt
{
    //Operations
    State Q = State();
    Eigen::Vector2d finq;

    ExecInt(SBot& Bot, )
    {
        Q = State();
        Q.q;
    }


};

#endif //TWOLINK_MANIP_EXECINTERFACE_H
