
#ifndef TWOLINK_MANIP_FKINEMATICS_H
#define TWOLINK_MANIP_FKINEMATICS_H

#include <Eigen/Dense>
#include <cmath>
#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/Sbot.h"

Eigen::Matrix3d FKin(s_bot& Bot, ExecInt& Operation);


#endif //TWOLINK_MANIP_FKINEMATICS_H
