#pragma once
#include <string>
#include <ranges>

#include <stdexcept>
#include <cstring>
#include <string_view>
#include <charconv>

namespace CFG {
    inline const float elThresholdDegrees = 5;
    inline const float azThresholdDegrees = 5;

    inline const int motorTimeoutMs = 500;
    inline const int relayPinForward = 17;
    inline const int relayPinReverse = 18;
    inline std::string mqttTopic = "solar/tracker";

    inline const char* ttyMotors = "/dev/ttyACM0";
    inline const char* ttyIMU = "/dev/ttyUSB0";

    //inline const char* mqtt = "tcp://broker.hivemq.com:1883";
     inline const char* mqttUri = "ssl://328ac36e4a744857a34cec7961c455fd.s1.eu.hivemq.cloud:8883";

    inline const char* mqttUser = "rpi5solar";
    inline const char* mqttPass = "Vc!Q4pf9Kku*WFW";
};

constexpr auto splitToVector = []( std::string_view view, char delimiter) {
    std::vector<std::string> result;
    for (auto&& subrange : view  | std::views::split(delimiter))  result.emplace_back(subrange.begin(), subrange.end());
    return result;
};

constexpr auto parseCommand = [](std::string_view input) {
    size_t p1 = input.find('|');
    size_t p2 = (p1 != std::string_view::npos) ? input.find('|', p1 + 1) : std::string_view::npos;

    std::string_view cmd;
    float f1 = 0.0f, f2 = 0.0f;

    if (p1 == std::string_view::npos) {
        cmd = input;
    } else {
        cmd = input.substr(0, p1);
        std::string_view s1 = input.substr(p1 + 1, (p2 != std::string_view::npos ? p2 : input.size()) - p1 - 1);
        std::from_chars(s1.data(), s1.data() + s1.size(), f1);

        if (p2 != std::string_view::npos) {
            std::string_view s2 = input.substr(p2 + 1);
            std::from_chars(s2.data(), s2.data() + s2.size(), f2);
        }
    }

    return std::tuple{cmd, f1, f2};
};