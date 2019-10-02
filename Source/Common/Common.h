
#ifndef TWOLINK_MANIP_COMMON_H
#define TWOLINK_MANIP_COMMON_H

#include <cmath>
#include <Eigen/Dense>
#include <vector>

Eigen::MatrixXd Rot(double& q);

Eigen::MatrixXd Transl(double x, double y);

Eigen::matrixXd FindPose(double& x, double& y, double& q);

std::vector<double> Coordiantes(Eigen::MatrixXd& Pose);




#endif //TWOLINK_MANIP_COMMON_H
