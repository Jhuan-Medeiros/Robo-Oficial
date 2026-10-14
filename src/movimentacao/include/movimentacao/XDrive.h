#ifndef MOVIMENTACAO_XDRIVE_H
#define MOVIMENTACAO_XDRIVE_H

#include <cmath>

class XDrive
{
private:
    double wheel_radius_ = 0.05;
    double robot_radius_ = 0.15;

public:
    XDrive() = default;

    void setRadius(double wheel_r, double robot_r)
    {
        wheel_radius_ = wheel_r;
        robot_radius_ = robot_r;
    }

    void getMotorSpeeds(double vx, double vy, double vth,
                        double &rpm0, double &rpm1, double &rpm2, double &rpm3)
    {
        const double sqrt3_2 = std::sqrt(3.0) / 2.0;

        double v0 = -sqrt3_2 * vx + 0.5 * vy + robot_radius_ * vth;
        double v1 =  sqrt3_2 * vx + 0.5 * vy + robot_radius_ * vth;
        double v2 = -1.0 * vy + robot_radius_ * vth;

        const double mps_to_rpm = 60.0 / (2.0 * M_PI * wheel_radius_);

        rpm0 = v0 * mps_to_rpm;
        rpm1 = v1 * mps_to_rpm;
        rpm2 = v2 * mps_to_rpm;
        rpm3 = 0.0;
    }

    void getRobotVelocity(double v0, double v1, double v2, double v3,
                          double &vx, double &vy, double &vth)
    {
        (void)v3;
        const double sqrt3 = std::sqrt(3.0);

        vx  = (v1 - v0) / sqrt3;
        vy  = (v0 + v1 - 2.0 * v2) / 3.0;
        vth = (v0 + v1 + v2) / (3.0 * robot_radius_);
    }
};

#endif // MOVIMENTACAO_XDRIVE_H