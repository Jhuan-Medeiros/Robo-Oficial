#ifndef KIWI_ROBOT_CONSTANTS_H
#define KIWI_ROBOT_CONSTANTS_H

#include <cmath>

namespace Constants
{
    constexpr double WHEEL_RADIUS = 0.0508;

    constexpr double ROBOT_RADIUS = 0.2000;

    constexpr double MAX_RPM = 100.0;

    constexpr char MOTOR0_CMD_TOPIC[] = "/titan/m_0/cmd";
    constexpr char MOTOR1_CMD_TOPIC[] = "/titan/m_1/cmd";
    constexpr char MOTOR2_CMD_TOPIC[] = "/titan/m_2/cmd";
    constexpr char MOTOR3_CMD_TOPIC[] = "/titan/m_3/cmd";

    constexpr char ENCODER0_TOPIC[] = "/titan/m_0/encoder";
    constexpr char ENCODER1_TOPIC[] = "/titan/m_1/encoder";
    constexpr char ENCODER2_TOPIC[] = "/titan/m_2/encoder";
    constexpr char ENCODER3_TOPIC[] = "/titan/m_3/encoder";

    constexpr char SERVO0_CMD_TOPIC[] = "/servo0/cmd";
}

#endif // KIWI_ROBOT_CONSTANTS_H