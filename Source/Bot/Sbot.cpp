
#include "Bot/Sbot.h"


SBot::SBot(){
    link_length = (Eigen::Vector2d(2) << 0.5, 0.5).finished();
    link_cm = (Eigen::Vector2d(2) << 0.25, 0.25).finished();
    mass = (Eigen::Vector2d(2) << 1.0, 1.0).finished();
    link_inertia = (Eigen::Vector2d(2, 1) << Inertia1, Inertia2).finished();
    joint_displaced = (Eigen::Vector2d(2) << 0.0, 0.0).finished();
    Amax = 8.0;
    Vmax = 10.0;
}

void SBot::set_linklengths(double a, double b)
{
    link_length << a, b;
    link_cm << a/2, b/2;
    link_inertia << mass * pow(a/2, 2), mass * pow(b/2, 2);
}


//void set_linkmasses(double a);
void SBot::set_linkmasses(double a)
{
    mass << a, a;
    link_inertia << a * pow(link_cm(0), 2), a * pow(link_cm(1), 2);
}


void SBot::set_acceleration(double A){
    Amax = A;
}


void SBot::set_velocity(double V){
    Vmax = V;
}