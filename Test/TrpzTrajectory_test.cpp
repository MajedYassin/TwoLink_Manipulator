#include <gtest/gtest.h>
#include "../Source/Solvers/TrpzTrajectory.h"
#include <iostream>



std::vector<Eigen::Vector2d> set_position()
 {
    Eigen::Vector2d q;
    std::vector<Eigen::Vector2d> matlabpos;
    Eigen::MatrixXd Q(64, 2);
    Q <<       0,         0,
         -0.0004,    0.0003,
         -0.0016,    0.0011,
         -0.0036,    0.0024,
         -0.0064,    0.0043,
         -0.0100,    0.0067,
         -0.0144,    0.0096,
         -0.0196,    0.0131,
         -0.0256,    0.0171,
         -0.0324,    0.0216,
         -0.0400,    0.0267,
         -0.0484,    0.0323,
         -0.0576,    0.0384,
         -0.0676,    0.0451,
         -0.0784,    0.0523,
         -0.0900,    0.0600,
         -0.1024,    0.0683,
         -0.1156,    0.0771,
         -0.1296,    0.0864,
         -0.1444,    0.0963,
         -0.1600,    0.1067,
         -0.1764,    0.1176,
         -0.1936,    0.1291,
         -0.2116,    0.1411,
         -0.2304,    0.1536,
         -0.2500,    0.1667,
         -0.2704,    0.1803,
         -0.2916,    0.1944,
         -0.3136,    0.2091,
         -0.3364,    0.2243,
         -0.3600,    0.2400,
         -0.3844,    0.2563,
         -0.3927,    0.2618,
         -0.4174,    0.2782,
         -0.4412,    0.2942,
         -0.4643,    0.3095,
         -0.4866,    0.3244,
         -0.5080,    0.3387,
         -0.5287,    0.3525,
         -0.5486,    0.3657,
         -0.5676,    0.3784,
         -0.5859,    0.3906,
         -0.6034,    0.4022,
         -0.6200,    0.4134,
         -0.6359,    0.4239,
         -0.6510,    0.4340,
         -0.6652,    0.4435,
         -0.6787,    0.4525,
         -0.6914,    0.4609,
         -0.7032,    0.4688,
         -0.7143,    0.4762,
         -0.7246,    0.4830,
         -0.7340,    0.4893,
         -0.7427,    0.4951,
         -0.7506,    0.5004,
         -0.7576,    0.5051,
         -0.7639,    0.5093,
         -0.7694,    0.5129,
         -0.7740,    0.5160,
         -0.7779,    0.5186,
         -0.7810,    0.5206,
         -0.7832,    0.5221,
         -0.7847,    0.5231,
         -0.7854,    0.5236;
    for(int i = 0; i != Q.rows(); ++i){
        matlabpos.emplace_back(Q.row(i));
    }
    return matlabpos;
}


std::vector<Eigen::Vector2d> set_accelerations()
{
    Eigen::Vector2d q;
    std::vector<Eigen::Vector2d> matlabacc;
    Eigen::MatrixXd Q(64, 2);
    Q <<       0,         0,
            -0.0004,    0.0003,
            -0.0016,    0.0011,
            -0.0036,    0.0024,
            -0.0064,    0.0043,
            -0.0100,    0.0067,
            -0.0144,    0.0096,
            -0.0196,    0.0131,
            -0.0256,    0.0171,
            -0.0324,    0.0216,
            -0.0400,    0.0267,
            -0.0484,    0.0323,
            -0.0576,    0.0384,
            -0.0676,    0.0451,
            -0.0784,    0.0523,
            -0.0900,    0.0600,
            -0.1024,    0.0683,
            -0.1156,    0.0771,
            -0.1296,    0.0864,
            -0.1444,    0.0963,
            -0.1600,    0.1067,
            -0.1764,    0.1176,
            -0.1936,    0.1291,
            -0.2116,    0.1411,
            -0.2304,    0.1536,
            -0.2500,    0.1667,
            -0.2704,    0.1803,
            -0.2916,    0.1944,
            -0.3136,    0.2091,
            -0.3364,    0.2243,
            -0.3600,    0.2400,
            -0.3844,    0.2563,
            -0.3927,    0.2618,
            -0.4174,    0.2782,
            -0.4412,    0.2942,
            -0.4643,    0.3095,
            -0.4866,    0.3244,
            -0.5080,    0.3387,
            -0.5287,    0.3525,
            -0.5486,    0.3657,
            -0.5676,    0.3784,
            -0.5859,    0.3906,
            -0.6034,    0.4022,
            -0.6200,    0.4134,
            -0.6359,    0.4239,
            -0.6510,    0.4340,
            -0.6652,    0.4435,
            -0.6787,    0.4525,
            -0.6914,    0.4609,
            -0.7032,    0.4688,
            -0.7143,    0.4762,
            -0.7246,    0.4830,
            -0.7340,    0.4893,
            -0.7427,    0.4951,
            -0.7506,    0.5004,
            -0.7576,    0.5051,
            -0.7639,    0.5093,
            -0.7694,    0.5129,
            -0.7740,    0.5160,
            -0.7779,    0.5186,
            -0.7810,    0.5206,
            -0.7832,    0.5221,
            -0.7847,    0.5231,
            -0.7854,    0.5236;
    for(int i = 0; i != Q.rows(); ++i){
        matlabacc.emplace_back(Q.row(i));
    }
    return matlabacc;
}

void check_trajectories(std::vector<Eigen::Vector2d>& result){
    for(int i = 0; i != result.size(); ++i) {
        for(int n = 0; n != 2; ++n) {
            std::cout << (result[i])(n) << std::endl;
        }
    }
}

void verify(std::vector<Eigen::Vector2d>& result, std::vector<Eigen::Vector2d>& answer){
    for(int i = 0; i != result.size(); ++i) {
        for(int n = 0; n != 2; ++n) {
            ASSERT_NEAR((answer[i])(n), (result[i])(n), 0.01);
        }
    }

}


TEST(SolverTest, RunTrajectoryPlanner)
{
    std::vector<Eigen::Vector2d> positions, velocities, accelerations;
    SBot bot;
    State s;
    TrapezTrajectory traj = TrapezTrajectory(bot, s);
    Eigen::Vector2d qf = (Eigen::Vector2d(2) << - M_PI_4, (M_PI_2)/3).finished();
    positions = traj.pos_traj(s.q, qf);
    velocities = traj.vel_traj();
    accelerations = traj.acc_traj();
    std::vector<Eigen::Vector2d> answer = set_position();
    std::cout << traj.A(0) << " " << traj.h(0) << "size of acc_traj" << accelerations.size() << std::endl;
    check_trajectories(positions);
    check_trajectories(accelerations);
    //verify(positions, answer);
    //std::cout << answer[60](0) << " " << positions[60](0)  << " size of computed traj : " << positions.size() << std::endl;
    //ASSERT_NEAR(answer[60](0), positions[60](0), 0.001);
}




