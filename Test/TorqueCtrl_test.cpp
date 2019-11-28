#include "../Source/Solvers/TorqueCtrl.h"
#include "../Source/Solvers/TrpzTrajectory.h"
#include "../Source/Bot/Sbot.h"
#include "../Source/Bot/State.h"
#include <gtest/gtest.h>
#include <iostream>


std::vector<Eigen::Vector2d> set_torque()
{
    std::vector<Eigen::Vector2d> matlabtorque;
    Eigen::MatrixXd Qt(63, 2);
    Qt <<    10.9256,    2.3981,
             10.9256,    2.3981,
             10.9256,    2.3981,
             10.9255,    2.3981,
             10.9254,    2.3982,
             10.9252,    2.3983,
             10.9247,    2.3986,
             10.9239,    2.3991,
             10.9227,    2.3999,
             10.9209,    2.4009,
             10.9184,    2.4024,
             10.9151,    2.4044,
             10.9107,    2.4071,
             10.9051,    2.4105,
             10.8981,    2.4148,
             10.8893,    2.4201,
             10.8787,    2.4266,
             10.8658,    2.4344,
             10.8505,    2.4437,
             10.8324,    2.4547,
             10.8112,    2.4676,
             10.7867,    2.4825,
             10.7584,    2.4998,
             10.7260,    2.5195,
             10.6891,    2.5420,
             10.6474,    2.5674,
             10.6004,    2.5960,
             10.5479,    2.6280,
             10.4892,    2.6638,
             10.4241,    2.7035,
             10.3521,    2.7474,
             10.2728,    2.7958,
             21.5188,    6.0413,
             27.2086,    7.6860,
             27.0404,    7.6655,
             26.8677,    7.6423,
             26.6915,    7.6166,
             26.5131,    7.5889,
             26.3333,    7.5595,
             26.1531,    7.5287,
             25.9736,    7.4969,
             25.7955,    7.4642,
             25.6198,    7.4311,
             25.4471,    7.3978,
             25.2782,    7.3644,
             25.1139,    7.3314,
             24.9548,    7.2989,
             24.8014,    7.2670,
             24.6544,    7.2361,
             24.5143,    7.2062,
             24.3815,    7.1777,
             24.2566,    7.1505,
             24.1398,    7.1249,
             24.0317,    7.1010,
             23.9325,    7.0790,
             23.8426,    7.0589,
             23.7621,    7.0408,
             23.6915,    7.0248,
             23.6308,    7.0111,
             23.5802,    6.9996,
             23.5399,    6.9904,
             23.5100,    6.9836,
             23.4906,    6.9791;
    for(int i = 0; i != Qt.rows(); ++i)
    {
        matlabtorque.emplace_back(Qt.row(i));
    }
    return matlabtorque;
}

void check_torque(std::vector<Eigen::Vector2d>& result, std::vector<Eigen::Vector2d>& answer)
{
    for(int i = 0; i != result.size(); ++i){
        for(int n = 0; n != 1; ++n){
            std::cout << result[i](n) << "  "  << result[i](n+1) <<  std::endl;
            //ASSERT_NEAR(result[i](n), answer[i](n), 0.1);
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
    TorqueController feedforward = TorqueController(s, bot);
    TrapezTrajectory traj = TrapezTrajectory(bot, s);
    Eigen::Vector2d qf = (Eigen::Vector2d(2) << - M_PI_4, (M_PI_2)/3).finished();
    positions = traj.pos_traj(s.q, qf);
    velocities = traj.vel_traj();
    accelerations = traj.acc_traj();
    torques = feedforward.feedforward_torque(positions, velocities, accelerations);
    std::vector<Eigen::Vector2d> example_torque = set_torque();

    std::cout << " break " << std::endl;
    std::cout << " " << std::endl;
    std::cout << "Compare Torque to Simulink Results"  << std::endl;

    check_torque(torques, example_torque);

}


