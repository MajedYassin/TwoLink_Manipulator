
#ifndef TWOLINK_MANIP_COMMON_H
#define TWOLINK_MANIP_COMMON_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <memory>


Eigen::Matrix2d Rot(double& q);

Eigen::MatrixX3d Transl(double& x, double& y);

Eigen::Matrix2d FindPose(double& x, double& y, double& q);

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

Eigen::MatrixXd derivative_array(Eigen::MatrixXd& array, double& timestep);


struct Integrator{

    Eigen::Vector2d y, y0;
    double dt;

    Integrator(Eigen::Vector2d& vec_0, double& interval);


    Eigen::Vector2d integral(Eigen::Vector2d& vec_i);

};






#endif //TWOLINK_MANIP_COMMON_H
