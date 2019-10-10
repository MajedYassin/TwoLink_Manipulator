
#ifndef TWOLINK_MANIP_FKINEMATICS_H
#define TWOLINK_MANIP_FKINEMATICS_H

#include "../Bot/State.h"
#include "../Bot/Sbot.h"
#include "../Common/Common.h"

namespace ForwardKinematics {
    Eigen::Matrix3d f_kin(SBot &bot, State &s, Eigen::Vector2d& qf);
}

#endif //TWOLINK_MANIP_FKINEMATICS_H
