#include "../Source/Solvers/TorqueCtrl.h"
#include "../Source/Solvers/TrpzTrajectory.h"
#include "../Source/Bot/Sbot.h"
#include "../Source/Bot/State.h"
#include <gtest/gtest.h>
#include <iostream>





void check_torque(std::vector<Eigen::Vector2d>& result, std::vector<Eigen::Vector2d>& answer)
{
    for(int i = 0; i != result.size(); ++i){
        for(int n = 0; n != 1; ++n){
            std::cout << result[i](n) << "  "  << result[i](n+1) <<  std::endl;
            //ASSERT_NEAR(result[i](n), answer[i](n), 0.01);
        }
    }
}


void read_array(std::vector<Eigen::Matrix2d>& array)
{
    for(int i = 0; i != array.size(); ++i) {
        for (int m = 0; m != array[i].rows(); ++m) {
            std::cout << array[i](m, 0) << "   " << array[i](m, 1) << std::endl;
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

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;
    std::cout << " Velocity trajectory array" << std::endl;
    check_torque(velocities, positions);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;

    //Checking variables for errors
    check_torque(feedforward.acceleration_response, positions);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;

    check_torque(feedforward.position_response_array, positions);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;

    std::cout << "Inertia Array "  << std::endl;
    read_array(feedforward.inertia_array);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;

    std::cout << "Coriolis Array "  << std::endl;
    check_torque(feedforward.coriolis_array, positions);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;

    std::cout << "Gravity Array "  << std::endl;
    check_torque(feedforward.gravity_array, positions);

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;


    check_torque(torques, positions);

}


