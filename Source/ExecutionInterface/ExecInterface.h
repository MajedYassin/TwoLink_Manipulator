
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
    State Q;
    double finq1;
    double finq2;

    ExecInt(SBot& Bot)
    {
        Q = State();
        Q.q;
    }




    if(Task == "Inverse Kinematics") finq1; finq2; //finq1 = InvK[1]; finq1 = InvK[2];
    if(Task == "Forward Kinematics" || Task == "Torque") finq1 = qf[1]; finq2 = qf[2];
};

#endif //TWOLINK_MANIP_EXECINTERFACE_H
