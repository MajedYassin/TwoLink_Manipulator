//
#include <gtest/gtest.h>
#include "../Source/Solvers/InvKinematics.h"
#include <Eigen/Dense>
#include "../Source/Common/Common.h"
#include <cmath>


TEST(SolverTest, RunForKinematics)
{
    SBot bot;
    Eigen::Matrix3d pose = (Eigen::Matrix3d(3, 3) << 0.0, 1.0, 0.0, -1.0, 0.0, -0.4, 0.0, 0.0, 1.0).finished();
    Coord c(pose);
    Eigen::Vector2d qf = (Eigen::Vector2d(2, 1) << M_PI_2, 0.0).finished();
    ASSERT_EQ(inv_kin(bot, pose, c), qf);
}
