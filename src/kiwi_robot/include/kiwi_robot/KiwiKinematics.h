#ifndef KIWI_ROBOT_KINEMATICS_H
#define KIWI_ROBOT_KINEMATICS_H

#include <cmath>
#include "kiwi_robot/Constants.h"

class KiwiKinematics
{
private:
    double wheel_radius_ = Constants::WHEEL_RADIUS;
    double robot_radius_ = Constants::ROBOT_RADIUS;

public:
    KiwiKinematics() = default;

    void setGeometry(double wheel_r, double robot_r)
    {
        wheel_radius_ = wheel_r;
        robot_radius_ = robot_r;
    }

    // Controla APENAS os motores na vida real:
    void inverseKinematics(double vx, double vy, double vth,
                           double &rpm0, double &rpm1, double &rpm2, double &rpm3) const
    {
        const double sqrt3_2 = std::sqrt(3.0) / 2.0;

        const double v0 = -sqrt3_2 * vx - 0.5 * vy - robot_radius_ * vth;
        const double v1 =  sqrt3_2 * vx - 0.5 * vy - robot_radius_ * vth;
        const double v2 =  0.0     * vx + 1.0 * vy - robot_radius_ * vth;

        const double mps_to_rpm = 60.0 / (2.0 * M_PI * wheel_radius_);

        rpm0 = v0 * mps_to_rpm;
        rpm1 = v1 * mps_to_rpm;
        rpm2 = v2 * mps_to_rpm;
        rpm3 = 0.0;
    }

    // Controla APENAS o movimento que o RViz vê (Odometria/TF):
    void forwardKinematics(double d0, double d1, double d2,
                           double &dx_body, double &dy_body, double &dth) const
    {
        const double sqrt3 = std::sqrt(3.0);

        dx_body = (d0 - d1) / sqrt3;                   // Frente/Trás no RViz
        dy_body = (d0 + d1 - 2.0 * d2) / 3.0;          // Esquerda/Direita no RViz
        dth     = (d0 + d1 + d2) / (3.0 * robot_radius_); // Giro no RViz
    }
};

#endif // KIWI_ROBOT_KINEMATICS_H