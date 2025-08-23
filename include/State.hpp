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

    static void handleStop() {
        if (trackerThread) trackerThread->stop();      // stop chasing thread
        state_ = TrackerState::Idle;
    }

    static void handleAuto() { 
        trackerThread = setInterval([] {
            std::cout << "chasing the SUN" << std::endl;
        }, 1000);
    }

    static void handleMoveTo(float az, float el) {
        if (state_.load() == TrackerState::Moving) return;
        auto [azDelta, elDelta] = SensorController::deltaTarget(az, el); 
        if (elDelta == 0 && azDelta == 0) return;
        state_ = TrackerState::Moving;

        if (elDelta < 0) Motors::moveD(); 
        else if (elDelta > 0) Motors::moveU();

        if (azDelta < 0) Motors::moveE(); 
        else if (azDelta > 0) Motors::moveW();

        std::thread([=]() {
            while(true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                auto [azDelta, elDelta] = SensorController::deltaTarget(az, el); 
                bool isMoving = state_.load() == TrackerState::Moving;
                if (elDelta == 0 || !isMoving) Motors::stopEl(); 
                if (azDelta == 0 || !isMoving) Motors::stopAz(); 
                if (!isMoving) break;

                if (!Motors::isAzMov && !Motors::isElMov) {
                    state_.store(TrackerState::Idle);
                    break;
                }
                std::cout << "az : " << azDelta << "\t\tel : " << elDelta << std::endl;
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