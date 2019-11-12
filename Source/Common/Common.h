
#ifndef TWOLINK_MANIP_COMMON_H
#define TWOLINK_MANIP_COMMON_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <fstream>


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

    Eigen::Vector2d y, y0, v0;
    double dt;

    Integrator(Eigen::Vector2d& vec_0, Eigen::Vector2d& var_0, double& interval);
    //var_0 initial state of variable that you are trying to integrate;
    //initial state of variable that you are integrating for, i.e the variable you are trying to find;


    Eigen::Vector2d integration(Eigen::Vector2d& vec_i);

    Eigen::Vector2d trapez_integration(Eigen::Vector2d& vec_i);

};


template<typename T, size_t R, size_t C>
inline void copy_to_document(std::fstream & doc, std::vector<Eigen::Matrix<T, R, C>> const & target_vec) {
    if (doc.is_open()) {
        for (auto const &vectors : target_vec) {
            for (auto const &elements : vectors) {
                doc << std::to_string(elements) + ", ";
            }
            doc << std::endl;
        }
    }
}

//std::fstream res_norm_file("/Users/Majed/Home/CLionProjects/Project_txt_files/torque_readings.csv",
//                              std::fstream::out | std::fstream::trunc);


#endif //TWOLINK_MANIP_COMMON_H
