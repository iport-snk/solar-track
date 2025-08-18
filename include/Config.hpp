#pragma once
#include <string>

namespace CFG {
    float elThresholdDegrees = 5;
    float azThresholdDegrees = 5;

    int motorTimeoutMs = 500;
    int relayPinForward = 17;
    int relayPinReverse = 18;
    std::string mqttTopic = "solar/tracker";
};