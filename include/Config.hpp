#pragma once
#include <string>

namespace CFG {
    inline const float elThresholdDegrees = 5;
    inline const float azThresholdDegrees = 5;

    inline const int motorTimeoutMs = 500;
    inline const int relayPinForward = 17;
    inline const int relayPinReverse = 18;
    inline std::string mqttTopic = "solar/tracker";

    inline const char* ttyMotors = "/dev/ttyACM0";
    inline const char* ttyIMU = "/dev/ttyUSB0";
};