#pragma once

#include "Mqtt.hpp"
#include "SerialWorker.h"
#include "Config.hpp"
#include <string>
#include <vector>
#include <ranges>
#include "BS_thread_pool.hpp"
#include "ImuController.hpp"
#include "Sun.hpp"
#include "DBG.hpp"


inline BS::thread_pool pool(6);

class CMD { 
            
    public:
        static void init() {
            SerialWorker::SEND("ELSTOP\n");
            SerialWorker::SEND("AZSTOP\n");
            if (!instance_) instance_ = std::make_unique<CMD>();
            DBG::log("[CMD] Arduino Azimuth Offset: ", getAzOffset());
        }
        static void loop() {
            static int sunTrackingSec = 0;
            if (++sunTrackingSec >= CFG::sunTrackingIntervalSecs) { 
                sunTrackingSec = 0;
                if (instance_->tracking) instance_->chaseTheSun();
            }
        }
        static void stopMoving(int axis) {
            SerialWorker::SEND(std::string( axis == 0 ? "EL" : "AZ") + "STOP\n");
            if (instance_->moving[axis]) instance_->moving[axis] = false;
        }   
        static std::string sunPosition() {
            auto [azimuth, elevation] = Sun::getSunPosition();
            return std::to_string(azimuth) + "~" + std::to_string(elevation);
        }
        static std::string move(std::string cmd) {
            SerialWorker::SEND(cmd + "\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return SerialWorker::cmd(std::string("MOTORS"));
        }
        static std::string poz() {
            auto az = SerialWorker::cmd("POZ");
            float rollVal = ImuController::getRoll();
            std::string roll = std::isnan(rollVal) ? "ERR" : std::to_string(static_cast<int>(rollVal));
            size_t sep = az.find('~');
            std::string azStr = (sep != std::string::npos) ? az.substr(sep + 1) : "ERR";
            return roll + "~" + azStr;
        }

        static std::string getAzOffset() {
            return SerialWorker::cmd("AZOFF");
        }

        static std::string setAzOffset(int offset) {
            return SerialWorker::cmd("AZOFF:" + std::to_string(offset));
        }

        static bool sensorErr(int axis, int poz) {
            return (axis == 1) && (poz < 10 || poz > 340);
        }

        static std::string move(int axis, int target, int postponeMs = 0) {
            if (postponeMs > 0) std::this_thread::sleep_for(std::chrono::milliseconds(postponeMs));
            DBG::log("[CMD] Move ", (axis == 0 ? "EL" : "AZ"), " to ", target);

            if (instance_->moving[axis]) return std::string("ALREADY MOVING ") + (axis == 0 ? "EL" : "AZ");
            std::string _r = poz();
            auto parts = splitToVector<int>(_r, '~');
            if (parts.size() <= static_cast<size_t>(axis)) return "ERR READING POSITION";
            int curr = parts[axis];
            if (std::abs(target - curr) < 3) return std::string("ALREADY AT POSITION ") + (axis == 0 ? "EL" : "AZ");
            if (sensorErr(axis, curr))  return "AZ SENSOR OUT OF RANGE";

            int prev = curr;
            int unchangedCount = 0;
            std::string dir = target > curr ? "CW" : "CCW";
            std::string cmd = std::string(axis == 0 ? "EL" : "AZ") + dir;
            SerialWorker::SEND(cmd + "\n");
            instance_->moving[axis] = true;
            while (instance_->moving[axis]) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                _r = poz();
                auto curParts = splitToVector<int>(_r, '~');
                if (curParts.size() <= static_cast<size_t>(axis)) continue;
                curr = curParts[axis];
                if (std::abs(curr - prev) < 2) { 
                    unchangedCount++;
                    std::cout << (axis == 0 ? "EL" : "AZ") << " Unchanged: " << curr << " : " << prev << std::endl;
                } else {
                    unchangedCount = 0;
                    prev = curr;
                }
                if (    (dir == "CW" && curr >= target) || 
                        (dir == "CCW" && curr <= target) || 
                        unchangedCount > 2 || sensorErr(axis, curr)
                    ) {
                    instance_->moving[axis] = false;
                    SerialWorker::SEND( (axis == 0 ? "ELSTOP\n" : "AZSTOP\n") );
                    if (sensorErr(axis, curr)) SerialWorker::SEND("AZ SENSOR OUT OF RANGE\n");
                }
            }
            return std::string("MOVING FINISHED ") + (axis == 0 ? "EL" : "AZ");
        }

        static std::string status() {
            std::string resp =  instance_->tracking ? (instance_->parking ? "PARKING" : "TRACKING") : "IDLE";
            resp += std::string("\n EL: ") + (instance_->moving[0] ? "MOVING" : "IDLE");
            resp += std::string("\n AZ: ") + (instance_->moving[1] ? "MOVING" : "IDLE");
            resp += std::string("\n IMU: ") + (ImuController::isHealthy() ? "OK" : "TIMEOUT");
            resp += std::string("\n AZ_OFF: ") + getAzOffset();
            resp += std::string("\n VER: ") + CFG::ver;
            return resp;
        }

        static void handleCommand(const std::string_view m_topic, const std::string_view m_payload ) {
            auto p = splitToVector<std::string>(m_topic, '/');
            if (p.size() < 3) return;

            // Support both "solar/cmd/<action>" (p[2]) and "solar/tracker/cmd/<action>" (after "cmd")
            size_t cmdIdx = 2;
            for (size_t i = 0; i < p.size(); ++i) {
                if (p[i] == "cmd" && i + 1 < p.size()) {
                    cmdIdx = i + 1;
                    break;
                }
            }
            std::string action = p[cmdIdx];

            int delay = 0;
            try {
                if (!m_payload.empty()) delay = std::stoi(std::string(m_payload));
            } catch (...) {
                delay = 0;
            }

            std::string resp = "";
            if (action == "auto") { instance_->tracking = true;  instance_->parking = false; }
            else if (action == "stop") { instance_->tracking = false; instance_->parking = false; }
            else if (action == "status") resp = CMD::status();
            else if (action == "azoff") {
                if (cmdIdx + 1 < p.size() && !p[cmdIdx + 1].empty()) {
                    try {
                        resp = "AZOFF: " + setAzOffset(std::stoi(p[cmdIdx + 1]));
                    } catch (...) {
                        resp = "AZOFF: ERR";
                    }
                } else if (!m_payload.empty() && m_payload != "?" && m_payload != "0") {
                    try {
                        resp = "AZOFF: " + setAzOffset(std::stoi(std::string(m_payload)));
                    } catch (...) {
                        resp = "AZOFF: ERR";
                    }
                } else {
                    resp = "AZOFF: " + getAzOffset();
                }
            } else {
                pool.detach_task( [p, cmdIdx, action, delay] () {
                    std::string resp = "";
                    if (action == "relays") resp = SerialWorker::cmd(std::string("MOTORS"));
                    else if (action == "poz") resp = poz();
                    else if (action == "sun") resp = sunPosition();
                    else if (action == "az" && cmdIdx + 1 < p.size()) {
                        try { resp = move(1, std::stoi(p[cmdIdx + 1]), delay); } catch (...) {}
                    }
                    else if (action == "el" && cmdIdx + 1 < p.size()) {
                        try { resp = move(0, std::stoi(p[cmdIdx + 1]), delay); } catch (...) {}
                    }
                    else if (action == "sel") stopMoving(0);
                    else if (action == "saz") stopMoving(1);
                    else if (CMD::mCmd.contains(action)) resp = move(CMD::mCmd.find(action)->second);

                    if (!resp.empty()) MqttClient::publish( resp );
                });
            }
            if (!resp.empty()) MqttClient::publish( resp );
        } 
    private:
        static void chaseTheSun() {
            if (instance_->moving[0] || instance_->moving[1]) return; // Don't chase while moving
            auto [azimuth, elevation] = Sun::getSunPosition();
            if (elevation < 4.0) {
                if (!instance_->parking) {
                    CMD::handleCommand("solar/cmd/el/0", "0"); // Move to parking position once
                    CMD::handleCommand("solar/cmd/az/180", "100"); 
                    std::cout << "[CMD] Sun below horizon, parking..." << std::endl;
                    instance_->parking = true;
                }
                return;
            }
            instance_->parking = false;
            elevation = 90.0 - elevation;
            elevation = elevation > CFG::elMaxDegrees ? CFG::elMaxDegrees : elevation;
            CMD::handleCommand("solar/cmd/el/" + std::to_string(static_cast<int>(elevation)), "0");
            CMD::handleCommand("solar/cmd/az/" + std::to_string(static_cast<int>(azimuth)), "100");
        }
        static inline std::unordered_map<std::string, std::string> mCmd = {
            {"mu", "ELCCW"}, {"md", "ELCW"}, {"me", "AZCCW"}, {"mw", "AZCW"} 
        };
        static inline std::unique_ptr<CMD> instance_ = nullptr; 
        bool tracking = false;
        bool parking = false;
        std::array<std::atomic<bool>, 2> moving = {false, false}; // moving[0]=El, moving[1]=Az
};
