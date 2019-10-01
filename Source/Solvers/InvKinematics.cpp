#include <cmath>
#include <Eigen/Dense>
#include <vector>


//Analytic Approach to Inverse Kinematics of 2D system
std::vector<int> InvK(double& a1, double& a2, int& x, int& y)
{
    std::vector<int> Qreq;
    int q2 = acos((x^2 + y^2 - a1^2 - a2^2)/(2* a1 * a2));
    int q1 = atan(y/x) - atan ((a2 * sin(q2))/(a1 + (a2 * cos(q2))));
    Qreq.push_back(q1);
    Qreq.push_back(q2);
    return Qreq;
}
