

#include "../Source/Solvers/TorqueCtrl.h"
#include <iostream>
#include <gtest/gtest.h>
#include <cmath>

void print_vec(std::vector<Eigen::Vector2d>& vector)
{
    for(int i =0; i != vector.size(); ++i){
        std::cout << vector[i](0) << " " << vector[i](1) << std::endl;
    }
}

TEST(SolverTest, PendulumTest)
{
    State s;
    SBot bot;
    Eigen::Vector2d initial_position = (Eigen::Vector2d(2) <<  M_PI_4, 0.0).finished();
    s.set_position(initial_position);
    TorqueController controller = TorqueController(s, bot);
    std::vector<Eigen::Vector2d> positions = controller.pendulum_test();
    print_vec(positions);
    std::cout << "   Break    "  << std::endl;
    std::cout << "   Break    "  << std::endl;

    std::vector<Eigen::Vector2d> velocity = controller.velocity_array;
    print_vec(velocity);
    /*
    std::cout << "   Break    "  << std::endl;
    std::cout << "   Break    "  << std::endl;
    std::vector<Eigen::Vector2d> acceleration = controller.acceleration_array;
    print_vec(acceleration);
     */
    //ASSERT_NEAR(add_operation(M_PI_2), b);
}

