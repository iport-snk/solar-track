#pragma once
#include <cstdint>
#include <optional>
#include <tuple>
#include "SensorController.h"
#include "IntervalRunner.hpp"
#include "Motors.hpp"
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
        
        if (p[1] == "move") { 
            auto params = splitToVector(payload, ',');
            float az = std::stof(params[1]);
            float el = std::stof(params[0]);
            TrackerFSM::handleMoveEl(el); 
        } else if (p[1] == "stop") TrackerFSM::handleStop();
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
        else if (       p[1] == "relays" )  resp = Motors::relays(); 

        return resp;
    };

    static void handleStop() {
        if (trackerThread) trackerThread->stop();      // stop chasing thread
        state_ = TrackerState::Idle;
    }

    static void handleAuto() { 
        trackerThread = setInterval([] {
            std::cout << "chasing the SUN" << std::endl;
        }, 1000);
    }

    static void handleMoveEl(float el) {
        if (state_.load() == TrackerState::Moving) return;
        float elDelta = SensorController::deltaEl(el); 
        if (elDelta == 0) return;
        state_ = TrackerState::Moving;

        if (elDelta < 0) Motors::moveD(); 
        else if (elDelta > 0) Motors::moveU();

        std::thread([=]() {
            while(true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                std::string relays = Motors::relays();
                auto elDelta = SensorController::deltaEl(el); 
                bool isMoving = state_.load() == TrackerState::Moving;
                if (elDelta == 0 || !isMoving) Motors::stopEl(); 
 
                if (!isMoving) break;

                if (!Motors::isAzMov && !Motors::isElMov) {
                    state_.store(TrackerState::Idle);
                    break;
                }
                std::cout  << "el : " << elDelta << std::endl;
            }
        }).detach();
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