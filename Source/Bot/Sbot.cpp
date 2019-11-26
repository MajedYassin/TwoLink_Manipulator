
#include "Bot/Sbot.h"

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