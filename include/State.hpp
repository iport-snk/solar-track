#pragma once
#include <cstdint>
#include <optional>
#include <tuple>
#include "SensorController.h"
#include "IntervalRunner.hpp"
#include "Motors.hpp"
#include "Globals.hpp"
// #include "Config.hpp"

enum class TrackerState : uint8_t {
    Idle,
    Moving,
    Error
};

class TrackerFSM {
public:
    static TrackerFSM& instance() {
        static TrackerFSM fsm;
        return fsm;
    }

    static std::string handleCmd(std::string_view topic, std::string_view payload) {
        std::cout << "[COMMAND] " << topic << " → " << payload << "\n";
        auto p = splitToVector(topic, '/');
        std::string resp = "";
        
        if (p[1] == "stop") TrackerFSM::handleStop();
        else if (p[1] == "auto") TrackerFSM::handleAuto();
        else if (       p[1] == "mu"     ) { Motors::moveU(); } 
        else if (       p[1] == "md"     ) { Motors::moveD(); } 
        else if (       p[1] == "me"     ) { Motors::moveE(); } 
        else if (       p[1] == "mw"     ) { Motors::moveW(); }
        else if (       p[1] == "sel"    )  Motors::stopEl();
        else if (       p[1] == "az"     )  Motors::az(p[2]);
        else if (       p[1] == "el"     )  Motors::el(p[2]);
        else if (       p[1] == "saz"    )  Motors::stopAz();
        else if (       p[1] == "poz"    )  resp = Motors::position();
        else if (       p[1] == "sun"    )  resp = TrackerFSM::sunPosition();
        else if (       p[1] == "relays" )  resp = Motors::relays(); 

        return resp;
    };

    static void handleStop() {
        if (trackerThread) trackerThread->stop();      // stop chasing thread
        state_ = TrackerState::Idle;
    }

    static std::string sunPosition() {
        auto [azimuth, elevation] = Sun::getSunPosition();
        return std::to_string(azimuth) + "~" + std::to_string(elevation);
    }

    static void handleAuto() { 
        trackerThread = setInterval([] {
            
            std::string state = (SerialWorker::CMD("MOTORS")).get();
            bool moving = (state[0] == '1' || state[1] == '1' || state[2] == '1' || state[3] == '1');
            if (moving) return;  // skip if already moving
            auto [azimuth, elevation] = Sun::getSunPosition();
            elevation = 90 - elevation; // convert to elevation from horizon
            if (azimuth > CFG::azMinDegrees && azimuth < CFG::azMaxDegrees) {
                auto az_str = SerialWorker::CMD("POZ").get();
                auto az = std::stoi(az_str.substr(az_str.find('~') + 1));
                if (std::abs(az - azimuth) > CFG::azThresholdDegrees)  Motors::az(std::to_string(static_cast<int>(azimuth)));
            }
            if (elevation > CFG::elMinDegrees && elevation < CFG::elMaxDegrees) {
                float roll = SensorController::getRoll();
                if (std::abs(roll - elevation) > CFG::elThresholdDegrees)  Motors::el(std::to_string(static_cast<int>(elevation)));
            }
 

        }, 20000);
    }

    static void onTargetReached() {

    }

    static void onFaultDetected(const char* reason) {

    }

    static TrackerState state() { return state_; }
    static std::tuple<double, double> target() { return target_; }

private:
    TrackerFSM() = default;
    static inline std::atomic<TrackerState> state_{TrackerState::Idle};
    static inline std::tuple<double, double> target_ = {0.0, 0.0};
    static inline std::unique_ptr<IntervalHandle> trackerThread;

};