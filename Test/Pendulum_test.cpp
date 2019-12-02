

#include "../Source/Solvers/TorqueCtrl.h"
#include "../Source/OperationInterface/OpInterface.h"
#include <iostream>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>

void print_vec(std::vector<Eigen::Vector2d>& vector)
{
    for(int i =0; i != vector.size(); ++i){
        std::cout << vector[i](0) << ", " << vector[i](1) << ", "  << std::endl;
    }
}

TEST(SolverTest, PendulumTest)
{
    State s;
    SBot bot;
    Eigen::Vector2d initial_position = (Eigen::Vector2d(2) <<  M_PI_4, 0.0).finished();
    s.set_position(initial_position);
    PendulumModel pendulum = PendulumModel(s, bot);
    std::vector<Eigen::Matrix<double, 2, 1>> positions, velocities;
    pendulum.release_pendulum();
    positions = pendulum.position_array;
    velocities = pendulum.velocity_array;

    print_vec(positions);
    //print_vec(velocities);

    std::fstream upload_file("/home/majed/CLionProjects/Project_txt_files/torquectrl_readings",
                           std::fstream::out | std::fstream::trunc);
    copy_to_document(upload_file, positions);
}

