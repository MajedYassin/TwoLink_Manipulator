//
#include <gtest/gtest.h>
#include "../Source/Solvers/FKinematics.h"
#include <Eigen/Dense>
#include <cmath>


struct SolverTest : public testing::Test{
    SolverTest();
};

TEST(SolverTest, RunForKinematics)
{
    State s;
    SBot bot;
    Eigen::Matrix3d answer = (Eigen::Matrix3d(3, 3) << 0.0, 1.0, 0.0, -1.0, 0.0, -0.4, 0.0, 0.0, 1.0).finished();
    Eigen::Vector2d qf = (Eigen::Vector2d(2, 1) << M_PI_2, 0.0).finished();
    ASSERT_EQ(answer, ForwardKinematics::f_kin(bot, s, qf));
}




