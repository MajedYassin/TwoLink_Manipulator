#include "InvKinematics.h"

//Analytic Approach to Inverse Kinematics of 2D system
Eigen::Vector2d inv_kin(SBot& bot, Eigen::Matrix3d& endpose)
{
    Coord cartesian = Coord(endpose);
    double a1, a2, x, y;
    a1 = bot.link_length(0);
    a2 = bot.link_length(1);
    x = cartesian.X;
    y = cartesian.Y;

    Eigen::Vector2d QReq;

    double q2 = acos((pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2))/(2* a1 * a2));
    double q1 = atan(y/x) - atan ((a2 * sin(q2))/(a1 + (a2 * cos(q2))));
    QReq(0) = q1;
    QReq(1) = q2;
    return QReq;
}


Eigen::Vector2d joint_angles(SBot& bot, double x, double y)
{

    double a1, a2;
    a1 = bot.link_length(0);
    a2 = bot.link_length(1);

    Eigen::Vector2d QReq;

    double q2 = acos((pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2))/(2* a1 * a2));
    double q1 = atan(y/x) - atan ((a2 * sin(q2))/(a1 + (a2 * cos(q2))));
    QReq(0) = q1;
    QReq(1) = q2;
    return QReq;
}
