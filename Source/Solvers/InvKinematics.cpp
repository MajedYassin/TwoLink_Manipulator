#include "InvKinematics.h"

//Analytic Approach to Inverse Kinematics of 2D system
std::vector<double> InvK(double& a1, double& a2, double& x, double& y)
{
    std::vector<double> Qreq;
    double q2 = acos((pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2))/(2* a1 * a2));
    double q1 = atan(y/x) - atan ((a2 * sin(q2))/(a1 + (a2 * cos(q2))));
    Qreq.push_back(q1);
    Qreq.push_back(q2);
    return Qreq;
}
