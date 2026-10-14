#ifndef XDRIVE_H
#define XDRIVE_H

#include <cmath>
#include "Constants.h"
#include "TitanQuad.h"

class XDrive {
private:
    double wheel_radius_;
    double robot_radius_;
    TitanQuad titan_config_;

public:
    XDrive(double wheel_radius = Constants::WHEEL_RADIUS, double robot_radius = Constants::ROBOT_RADIUS)
    : wheel_radius_(wheel_radius), robot_radius_(robot_radius) {}

    void setRadius(double wheel_radius, double robot_radius) {
        wheel_radius_ = wheel_radius;
        robot_radius_ = robot_radius;
    }

    void setCmdInversions(double inv1, double inv2, double inv3, double inv4) {
        titan_config_.setCmdInversions(inv1, inv2, inv3, inv4);
    }

    void setEncInversions(double inv1, double inv2, double inv3, double inv4) {
        titan_config_.setEncInversions(inv1, inv2, inv3, inv4);
    }

    void setGains(double g1, double g2, double g3, double g4) {
        titan_config_.setGains(g1, g2, g3, g4);
    }

    void Drive(double vx, double vy, double vth, double &rpm1, double &rpm2, double &rpm3, double &rpm4) {
        double sqrt2_2 = std::sqrt(2.0) / 2.0;

        double v1_lin = sqrt2_2 * (vx - vy) + (robot_radius_ * vth);
        double v2_lin = sqrt2_2 * (vx + vy) + (robot_radius_ * vth);
        double v3_lin = sqrt2_2 * (-vx + vy) + (robot_radius_ * vth);
        double v4_lin = sqrt2_2 * (-vx - vy) + (robot_radius_ * vth);

        rpm1 = (v1_lin / (2.0 * M_PI * wheel_radius_)) * 60.0 * titan_config_.motor1.cmd_inversion * titan_config_.motor1.gain;
        rpm2 = (v2_lin / (2.0 * M_PI * wheel_radius_)) * 60.0 * titan_config_.motor2.cmd_inversion * titan_config_.motor2.gain;
        rpm3 = (v3_lin / (2.0 * M_PI * wheel_radius_)) * 60.0 * titan_config_.motor3.cmd_inversion * titan_config_.motor3.gain;
        rpm4 = (v4_lin / (2.0 * M_PI * wheel_radius_)) * 60.0 * titan_config_.motor4.cmd_inversion * titan_config_.motor4.gain;
    }

    void getRobotVelocity(double w1, double w2, double w3, double w4, double &vx, double &vy, double &vth) {
        double v1 = (w1 * titan_config_.motor1.enc_inversion) * wheel_radius_;
        double v2 = (w2 * titan_config_.motor2.enc_inversion) * wheel_radius_;
        double v3 = (w3 * titan_config_.motor3.enc_inversion) * wheel_radius_;
        double v4 = (w4 * titan_config_.motor4.enc_inversion) * wheel_radius_;

        double sqrt2_4 = std::sqrt(2.0) / 4.0;

        vx = sqrt2_4 * (v1 + v2 - v3 - v4);
        vy = sqrt2_4 * (-v1 + v2 + v3 - v4);
        vth = (1.0 / (4.0 * robot_radius_)) * (v1 + v2 + v3 + v4);
    }
};

#endif