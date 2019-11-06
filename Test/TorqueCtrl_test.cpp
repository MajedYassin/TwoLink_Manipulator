#include "../Source/Solvers/TorqueCtrl.h"
#include "../Source/Solvers/TrpzTrajectory.h"
#include "../Source/Bot/Sbot.h"
#include "../Source/Bot/State.h"
#include <gtest/gtest.h>
#include <iostream>





void check_torque(std::vector<Eigen::Vector2d> result, std::vector<Eigen::Vector2d> answer)
{
    for(int i = 0; i != result.size(); ++i){
        for(int n = 0; n != 2; ++n){
            std::cout << result[i](n) << std::endl;
            //ASSERT_NEAR(result[i](n), answer[i](n), 0.01);
        }
    }


}


TEST(SolverTest, TestTorqueControl)
{
    std::vector<Eigen::Vector2d> positions, velocities, accelerations, torques;
    SBot bot;
    State s;
    InvDynamics feedforward = InvDynamics(s, bot);
    TrapezTrajectory traj = TrapezTrajectory(bot, s);
    Eigen::Vector2d qf = (Eigen::Vector2d(2) << - M_PI_4, (M_PI_2)/3).finished();
    positions = traj.pos_traj(s.q, qf);
    velocities = traj.vel_traj();
    accelerations = traj.acc_traj();
    torques = feedforward.feedforward_torque(positions, velocities, accelerations);
    check_torque(feedforward.acceleration_response, positions);
    std::cout << " break " << std::endl;
    check_torque(torques, positions);

}


