#include "Common.h"

//Common mathematical functions and operation used in the Solvers

//Can Use Eigen::Rotation2D<double> rot2(double& q);
Eigen::Matrix2d Rot(double q)
{
    double A11 = cos(q);
    double A12 = sin(q);
    double A21 = -1 * A12;
    double A22 = A11;
    Eigen::Matrix2d M(2, 2);
    M << A11, A12, A21, A22;
    return M;
}
//Can Use Eigen::Translation<double,2>(tx, ty)
Eigen::Matrix3d Transl(double x, double y)
{
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    T(0, 2) = x;
    T(1, 2) = y;
    return T;
}

Eigen::Matrix3d FindPose(double& x, double& y, double& q)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.block<2,2>(0, 0) = Rot(q);
    return R * Transl(x, y);
}




std::vector<Eigen::Vector2d> derivative_array(std::vector<Eigen::Vector2d>& array, double& timestep)
{
    auto length = array.size();
    std::vector<Eigen::Vector2d> diff_array;
    int j = 0;

    for (int i = 1; i != length; ++i)
    {
        j = i - 1;
        diff_array.emplace_back((array[i] - array[j]) / timestep);
    }
    return diff_array;
}


Integrator::Integrator(Eigen::Vector2d& vec_0, double& interval)
{
    y0 = Eigen::Vector2d::Zero();
    y = vec_0;
    dt = interval;
}

Eigen::Vector2d Integrator::integral(Eigen::Vector2d& vec_i){
    y = (vec_i * dt) + y0;
    y0 = y;
    return y;
}
