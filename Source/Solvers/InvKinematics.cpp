#include "InvKinematics.h"

//Analytic Approach to Inverse Kinematics of 2D system
Eigen::Vector2d InvK(s_bot& Bot, ExecInt& Input)
{
    Coordinates Cartesian = Coordinates{Input.EPose};
    double a1, a2, x, y;
    a1 = Bot.a1;
    a2 = Bot.a2;
    x = Cartesian.X;
    y = Cartesian.Y;

    Eigen::Vector2d QReq;

    double q2 = acos((pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2))/(2* a1 * a2));
    double q1 = atan(y/x) - atan ((a2 * sin(q2))/(a1 + (a2 * cos(q2))));
    QReq(1) = q1;
    QReq(2) = q2;
    return QReq;
}
