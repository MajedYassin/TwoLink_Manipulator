
#ifndef TWOLINK_MANIP_INVKINEMATICS_H
#define TWOLINK_MANIP_INVKINEMATICS_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"

Eigen::Vector2d inv_kin(SBot& bot, Eigen::Matrix3d& endpose);

Eigen::Vector2d joint_angles(SBot& bot, double x, double y);

#endif //TWOLINK_MANIP_INVKINEMATICS_H
