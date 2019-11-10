#include "VelocityKinematics.h"


VelKinematics::VelKinematics(SBot& parameters, State& state) : bot(parameters), instance(state)
{
    //Map Component: retunrs the component of the Jacobian for joint configuration q
    Component[X] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double jac_x;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double q_sum = 0.0;
                while (m <= n) q_sum += q(m); ++m;
                if (m == l) jac_x = bot.link_cm(m) * sin(q_sum);
                else jac_x = bot.link_length(m) * sin(q_sum);
                Jac(0, i) -= jac_x;
            }
        }
        return Jac;
    };
    Component[Y] = [&] (Eigen::Matrix2d Jac, Eigen::Vector2d q, int l) -> Eigen::Matrix2d
    {
        double jac_y = 0;
        for(int i = 0; i <= l; ++i){
            for(int n = i; n <= l; ++n){
                int m = i;
                double q_sum = 0.0;
                while(m <= n) {
                    q_sum += q(m);
                    ++m;
                }
                if (m == l) jac_y = bot.link_cm(n) * cos(q_sum);
                else jac_y = bot.link_length(n) * cos(q_sum);
                Jac(1, i) -= jac_y;
            }

        }
        return Jac;
    };
}



Eigen::Matrix2d VelKinematics::get_jacobian(Eigen::Vector2d& q, int link)
{
    Eigen::Matrix2d J = Eigen::Matrix2d::Zero();
    //Two Rows as the manipulator end-effector has only 2DOF

    J = Component[X](J, q, link);
    J = Component[Y](J, q, link);

    return J;
}


Eigen::Vector2d VelKinematics::get_endeffector_velocity(Eigen::Vector2d& joint_velocity)
{
    return get_jacobian(instance.q, joint_velocity.size()) * joint_velocity;
}

std::vector<Eigen::Vector2d> VelKinematics::end_effector_velocities(std::vector<Eigen::Vector2d>& position_array, std::vector<Eigen::Vector2d>& velocity_array)
{
    std::vector<Eigen::Vector2d> ee_velocity; // end-effector velocity array
    Eigen::Vector2d velocity_i; //end-effector velocity at instance i

    for(int i= 0; i != position_array.size(); ++i)
    {
        velocity_i = get_jacobian(position_array[i], 2) * velocity_array[i];
        ee_velocity.emplace_back(velocity_i);
    }

    return ee_velocity;
}