#include "Common.h"

//Common mathematical functions and operation used in the Solvers

//Can Use Eigen::Rotation2D<double> rot2(double& q);
Eigen::MatrixXd Rot(double& q)
{
    double A11=cos(q);
    double A21 = sin(q);
    double A12 = - A21;
    double A22 = A11;
    Eigen::MatrixXd M;
    M << A11, A12,
        A21, A22;
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

Eigen::MatrixX2d derivative_array(Eigen::MatrixX2d& array, double& timestep)
{
    auto length= array.rows();
    Eigen::MatrixX2d diff_array;
    int j = 0;
    while(j != 2)
    {
        for (int i = 0; i < length; ++i)
        {
            diff_array(i,j) = (array(i+1, j)- array(i, j))/ timestep;
        }
        ++j;
    }
    return diff_array;
}


Integrator::Integrator(Eigen::VectorXd& vec_0, double& interval)
{
    y0 = std::make_unique<Eigen::VectorXd>(5, 1);
    *y = vec_0;
    y  = std::make_unique<Eigen::VectorXd>(5, 1);
    dt = interval;
}

Eigen::VectorXd Integrator::integral(Eigen::VectorXd& vec_i){
    *y = (vec_i * dt) + *y0;
    *y0 = *y;
    return *y;
}
