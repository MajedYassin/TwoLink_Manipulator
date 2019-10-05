#include "Common.h"

//Common mathematical functions and operation used in the Solvers

//Can Use Eigen::Rotation2D<double> rot2(double& q);
Eigen::Matrix3d Rot(double& q)
{
    double A11=cos(q);
    double A21 = sin(q);
    double A12 = - A21;
    double A22 = A11;
    Eigen::Matrix3d M;
    M << A11, A12, 0.0,
        A21, A22, 0.0,
        0.0, 0.0, 0.0;
    return M;
}
//Can Use Eigen::Translation<double,2>(tx, ty)
Eigen::Matrix3d Transl(double& x, double& y)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(1, 3) = x;
    T(2, 3) = y;
    return T;
}

Eigen::Matrix3d FindPose(double& x, double& y, double& q)
{
    return Rot(q) * Transl(x, y);
}


template <typename T>
void Print_vector(T& vector){
    for( int i = 0; i != vector.size(); ++i)
    {
        std::cout << vector(i) << std::endl;
    }
}

template <typename T>
void Print(T& var){
    std::cout << var << std::endl;
}


