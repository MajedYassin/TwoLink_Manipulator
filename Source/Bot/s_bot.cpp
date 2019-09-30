#include 's_bot.h'

Struct s_bot{
    //Link Lengths
    const double a1;
    const double a2;
    const double cm1;
    const double cm2;
    //Link Masses
    const double m1;
    const double m2;
    //Initial Joint Angles
    signed int inq1;
    signed int inq2;
    //desired/required joint angles
    signed int finq1;
    signed int finq2;
    //Matrices

    Eigen::Matrix3d Inert_M;
    Eigen::Matrix<double, 1, 2> Grav_M;
    Eigen::Matrix3d Cor_M;

    //Inverse and Forward Kinematics


    //Dynamics






    s_bot(){

    }



};