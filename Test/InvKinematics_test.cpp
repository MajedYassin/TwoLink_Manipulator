//
#include <gtest/gtest.h>
#include "../Source/Solvers/InvKinematics.h"
#include <Eigen/Dense>
#include <cmath>

void check_ikine(Eigen::Vector2d& answer, Eigen::Vector2d& q){
    for(int i = 0; i != q.size(); ++i) {
        std::cout << q(i) << std::endl;
        ASSERT_NEAR(answer(i), q(i), 0.01);
    }
}

TEST(SolverTest, RunForKinematics)
{
    SBot bot;
    Eigen::Matrix3d pose = (Eigen::Matrix3d(3, 3) << 0.0, 1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0).finished();
    Coord c = Coord();
    Eigen::Vector2d qf = (Eigen::Vector2d(2, 1) << - M_PI_2, 0.0).finished();
    Eigen::Vector2d answer = inv_kin(bot, pose, c);
    check_ikine(answer, qf);
}
