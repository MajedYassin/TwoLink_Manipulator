//
#include <gtest/gtest.h>
#include "../Source/Solvers/FKinematics.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>


void check_matrix(Eigen::Matrix3d& matrix, Eigen::Matrix3d& answer)
{
    for(int n = 0; n != matrix.rows(); ++n){
        for(int m = 0; m != matrix.cols(); ++m){
            std::cout << matrix(n, m) << std::endl;
            ASSERT_NEAR(matrix(n, m), answer(n, m), 0.01);
        }
    }
}


TEST(SolverTest, RunForKinematics)
{
    State s;
    SBot bot;
    double pi_2 = M_PI_2;
    Eigen::Matrix3d answer = (Eigen::Matrix3d(3, 3) << 0.0, 1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0).finished();
    Eigen::Vector2d qf = (Eigen::Vector2d(2) << pi_2, 0.0).finished();
    Eigen::Matrix3d result = ForwardKinematics::f_kin(bot, s, qf);
    check_matrix(result, answer);
    Eigen::Matrix3d error = Eigen::Matrix3d::Constant(0.01);
}




