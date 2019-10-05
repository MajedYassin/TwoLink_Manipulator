
#ifndef TWOLINK_MANIP_FKINEMATICS_H
#define TWOLINK_MANIP_FKINEMATICS_H

#include "../Bot/State.h"
#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/Sbot.h"

namespace ForwardKinematics {
    Eigen::Matrix3d f_kin(SBot &Bot, State &Input);
}

#endif //TWOLINK_MANIP_FKINEMATICS_H
