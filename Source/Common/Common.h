
#ifndef TWOLINK_MANIP_COMMON_H
#define TWOLINK_MANIP_COMMON_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>


Eigen::Matrix3d Rot(double& q);

Eigen::Matrix3d Transl(double& x, double& y);

Eigen::Matrix3d FindPose(double& x, double& y, double& q);

struct Coordinates{
    double X, Y;
    Coordinates(Eigen::Matrix3d& M){
        X = M(1, 3);
        Y = M(2, 3);
    }

};

template <typename T>
void Print(T& var);

template <typename T>
void Print_vector(T& vector);

Eigen::MatrixX2d derivative_array(Eigen::MatrixX2d& array, double& timestep);





#endif //TWOLINK_MANIP_COMMON_H
