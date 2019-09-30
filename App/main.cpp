#include <iostream>
#include <Eigen/Dense>
#include <string>


int main() {
    std::cout << "Hello, World!" << std::endl;
    //Dynamic matrix - resizable
    Eigen::MatrixXd d;
    //Fixed size matrix
    Eigen::Matrix3d f;

    //ind size of a matrix
    f.size();

    std::cout << f.size() << std::endl;

    //resizing a dynamic matrix
    d.resize(4, 6);

    //initialising a matrix
    f << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    //or
    f = Eigen::Matrix3d::Random();
    d = Eigen::MatrixXd::Random(5, 5);

    std::cout << f << std::endl;
    std::cout << d << std::endl;

    return 0;
}