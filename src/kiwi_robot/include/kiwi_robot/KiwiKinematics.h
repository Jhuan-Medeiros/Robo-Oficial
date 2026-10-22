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

    // Cinemática Inversa com frente invertida (vx e vy negativados)
    void inverseKinematics(double vx, double vy, double vth,
                           double &rpm0, double &rpm1, double &rpm2, double &rpm3) const
    {
        const double sqrt3_2 = std::sqrt(3.0) / 2.0;

        const double inv_vx = -vx;
        const double inv_vy = -vy;

        const double v0 =  sqrt3_2 * inv_vx + 0.5 * inv_vy + robot_radius_ * vth;
        const double v1 = -sqrt3_2 * inv_vx + 0.5 * inv_vy + robot_radius_ * vth;
        const double v2 =  0.0     * inv_vx - 1.0 * inv_vy + robot_radius_ * vth;

        const double mps_to_rpm = 60.0 / (2.0 * M_PI * wheel_radius_);

        rpm0 = v0 * mps_to_rpm;
        rpm1 = v1 * mps_to_rpm;
        rpm2 = v2 * mps_to_rpm;
        rpm3 = 0.0;
    }

    // Cinemática Direta com frente invertida para a odometria acompanhar
    void forwardKinematics(double d0, double d1, double d2,
                           double &dx_body, double &dy_body, double &dth) const
    {
        const double sqrt3 = std::sqrt(3.0);

        dx_body = -((d0 - d1) / sqrt3);
        dy_body = -((d0 + d1 - 2.0 * d2) / 3.0);
        dth     = (d0 + d1 + d2) / (3.0 * robot_radius_);
    }
};

#endif // KIWI_ROBOT_KINEMATICS_H
