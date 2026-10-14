#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
    constexpr double WHEEL_RADIUS = 0.0508;
    constexpr double ROBOT_RADIUS = 0.156; 

    constexpr double MAX_RPM = 100.0;
    constexpr double MAX_RPM_SAFE = MAX_RPM * 1.2;

    constexpr double DEFAULT_FATOR_LINEAR = 1.2;
    constexpr double DEFAULT_FATOR_ANGULAR = 1.4;

    constexpr char MOTOR0_CMD_TOPIC[] = "/titan/m_0/cmd";
    constexpr char MOTOR1_CMD_TOPIC[] = "/titan/m_1/cmd";
    constexpr char MOTOR2_CMD_TOPIC[] = "/titan/m_3/cmd";
    constexpr char MOTOR3_CMD_TOPIC[] = "/titan/m_2/cmd";

    constexpr char ENCODER0_RPM_TOPIC[] = "/titan/m_0/rpm";
    constexpr char ENCODER1_RPM_TOPIC[] = "/titan/m_1/rpm";
    constexpr char ENCODER2_RPM_TOPIC[] = "/titan/m_3/rpm";
    constexpr char ENCODER3_RPM_TOPIC[] = "/titan/m_2/rpm";

    constexpr char SERVO_GARRA_TOPIC[] = "/servo0/cmd";
}

#endif