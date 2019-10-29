
#ifndef TWOLINK_MANIP_COMMON_H
#define TWOLINK_MANIP_COMMON_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <memory>


Eigen::Matrix2d Rot(double q);

Eigen::Matrix3d Transl(double x, double y);

Eigen::Matrix3d FindPose(double& x, double& y, double& q);

struct Coord{
    double X, Y;

    explicit Coord(Eigen::Matrix3d& M){
        X = M(0, 2);
        Y = M(1, 2);
    }

    Coord(){

    }

};

std::vector<Eigen::Vector2d> derivative_array(std::vector<Eigen::Vector2d>& array, double& timestep);


struct Integrator{

    Eigen::Vector2d y, y0;
    double dt;

    Integrator(Eigen::Vector2d& vec_0, double& interval);


    Eigen::Vector2d integral(Eigen::Vector2d& vec_i);

};






#endif //TWOLINK_MANIP_COMMON_H
