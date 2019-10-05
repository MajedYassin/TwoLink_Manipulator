
#ifndef TWOLINK_MANIP_INVKINEMATICS_H
#define TWOLINK_MANIP_INVKINEMATICS_H

#include "../Bot/State.h"
#include "../ExecutionInterface/ExecInterface.h"
#include "../Bot/Sbot.h"

Eigen::Vector2d inv_kin(SBot& bot, State& input);

#endif //TWOLINK_MANIP_INVKINEMATICS_H
