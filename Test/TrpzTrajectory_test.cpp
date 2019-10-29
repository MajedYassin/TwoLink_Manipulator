#include <gtest/gtest.h>
#include "../Source/Solvers/TrpzTrajectory.h"
#include <Eigen/Dense>
#include <cmath>
#include <array.h>

void check_trajectories(std::array<Eigen::Vector2d, 3>& answer, std::vector<Eigen::Vector2d>& result){
    for(int i = 0; i != result.size(); ++i) {
        std::cout << result(i) << std::endl;
        ASSERT_NEAR(answer[i, 0], result(i), 0.01);
    }
}

TEST(SolverTest, RunTrajectoryPlanner)
{
    std::vector<Eigen::Vector2d> positions, velocities, accelerations;
    std::array<Eigen::Vector2d, 3> answer;
    SBot bot;
    State s;
    TrapezTrajectory traj = TrapezTrajectory(bot, s);
    Eigen::Vector2d qf = (Eigen::Vector2d(2, 1) << -M_PI_2, M_PI_2).finished();
    positions = traj.pos_traj(s.q, qf);
    velocities = traj.vel_traj();
    accelerations = traj.acc_traj();

    check_trajectories(positions, velocities, accelerations);
}
