#ifndef TITAN_QUAD_H
#define TITAN_QUAD_H

struct MotorConfig {
    double cmd_inversion = 1.0;
    double enc_inversion = 1.0;
    double gain = 1.0;
};

class TitanQuad {
public:
    MotorConfig motor1;
    MotorConfig motor2;
    MotorConfig motor3;
    MotorConfig motor4;

    TitanQuad() = default;

    void setCmdInversions(double inv1, double inv2, double inv3, double inv4) {
        motor1.cmd_inversion = inv1;
        motor2.cmd_inversion = inv2;
        motor3.cmd_inversion = inv3;
        motor4.cmd_inversion = inv4;
    }

    void setEncInversions(double inv1, double inv2, double inv3, double inv4) {
        motor1.enc_inversion = inv1;
        motor2.enc_inversion = inv2;
        motor3.enc_inversion = inv3;
        motor4.enc_inversion = inv4;
    }

    void setGains(double g1, double g2, double g3, double g4) {
        motor1.gain = g1;
        motor2.gain = g2;
        motor3.gain = g3;
        motor4.gain = g4;
    }
};

#endif