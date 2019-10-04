
#ifndef TWOLINK_MANIP_INVKINEMATICS_H
#define TWOLINK_MANIP_INVKINEMATICS_H

#include <cmath>
#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/Sbot.h"

Eigen::Vector2d InvK(s_bot& Bot, ExecInt& Input);

#endif //TWOLINK_MANIP_INVKINEMATICS_H
