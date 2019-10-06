#include <iostream>
#include <Eigen/Dense>


int main()
{
    std::cout << "Two Link Manipulator Robot"  << std::endl;
    Eigen::ArrayX2d array;
    array(0) = 9;
    array(1) = 12;
    array(2) = 14;
    auto size = array.size();

    std::cout << size << std::endl;



    return 0;
}