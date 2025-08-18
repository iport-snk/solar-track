#pragma once
#include <chrono>
#include <ctime>
#include <tuple>
#include "spa.h" // Your original SPA header

namespace Sun {

inline std::tm getUtcTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    return *gmtime(&tt);
}

inline std::tuple<float, float> getSunPosition() {
    std::tm utc = getUtcTime();

    spa_data spa{};
    spa.year          = utc.tm_year + 1900;
    spa.month         = utc.tm_mon + 1;
    spa.day           = utc.tm_mday;
    spa.hour          = utc.tm_hour;
    spa.minute        = utc.tm_min;
    spa.second        = utc.tm_sec;

    spa.timezone      = 0.0;           // UTC
    spa.delta_ut1     = 0.0;
    spa.delta_t       = 69.0;          // Approx for 2025
    spa.longitude     = 31.082747;
    spa.latitude      = 50.434988;
    spa.elevation     = 150.0;
    spa.pressure      = 1013.25;
    spa.temperature   = 20.0;
    spa.atmos_refract = 0.5667;
    spa.function      = SPA_ZA;

    int result = spa_calculate(&spa);
    if (result == 0) {
        return {spa.azimuth, spa.e};
    } else {
        return {-1.0, -1.0}; // Sentinel values for error
    }
}

}