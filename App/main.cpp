#include <iostream>
#include <Eigen/Dense>
#include "../Test/FKinematics_test.cpp"



int main(int argc, char* argv[])
{
    std::cout << "Two Link Manipulator Robot"  << std::endl;

    testing::InitGoogleTest(&argc, argv);

    RUN_ALL_TESTS();

    return 0;
}