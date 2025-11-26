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


inline BS::thread_pool pool;

class CMD { 
            
    public:
        static void init() {
            SerialWorker::SEND("ELSTOP\n");
            SerialWorker::SEND("AZSTOP\n");
            if (!instance_) instance_ = std::make_unique<CMD>();
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
            std::string roll = std::to_string(static_cast<int>(ImuController::getRoll()));
            az = az.substr(az.find('~') + 1);
            return roll + "~" + az;
        }

        static std::string move(int axis, float target) { 
            if (instance_->moving[axis]) return std::string("ALREADY MOVING ") + (axis == 0 ? "EL" : "AZ");
            std::string _r = poz();
            int curr = (splitToVector<int>(_r, '~'))[axis];
            if (std::abs(target - curr) < 3) return std::string("ALREADY AT POSITION ") + (axis == 0 ? "EL" : "AZ");
            int prev = curr;
            int unchangedCount = 0;
            
            std::string cmd = std::string(axis == 0 ? "EL" : "AZ") + (target > curr ? "CW" : "CCW");
            SerialWorker::SEND(cmd + "\n");
            instance_->moving[axis] = true;
            while (instance_->moving[axis]) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                _r = poz();
                curr = (splitToVector<int>(_r, '~'))[axis];
                if (std::abs(curr - prev) < 2) { 
                    unchangedCount++;
                    std::cout << "Unchanged: " << curr << " : " << prev << std::endl;
                } else {
                    unchangedCount = 0;
                    prev = curr;
                }
                if ( (target > curr && (curr + 1) >= target) || (target < curr && (curr - 1) <= target) || unchangedCount > 2 ) {
                    instance_->moving[axis] = false;
                    SerialWorker::SEND( (axis == 0 ? "ELSTOP\n" : "AZSTOP\n") );
                }
            }
            return std::string("MOVING FINISHED ") + (axis == 0 ? "EL" : "AZ");
        }

        static void handleCommand(const std::string_view m_topic, const std::string_view m_payload ) {
            auto p = splitToVector<std::string>(m_topic, '/');
            
            if (        p[2] == "auto") { instance_->tracking = true;  instance_->parking = false; }
            else if (   p[2] == "stop") { instance_->tracking = false; instance_->parking = false; }
            else if (   p[2] == "status") {
                    std::string resp =  instance_->tracking ? (instance_->parking ? "PARKING" : "TRACKING") : "IDLE";
                    resp += std::string("\n EL:") + (instance_->moving[0] ? "MOVING" : "IDLE");
                    resp += std::string("\n AZ:") + (instance_->moving[1] ? "MOVING" : "IDLE");
                    MqttClient::publish( resp );

            }
            else {
                pool.detach_task( [p] () {
                std::string resp = "";
                if (        p[2] == "relays")       resp = SerialWorker::cmd(std::string("MOTORS"));
                else if (   p[2] == "poz")          resp = poz();
                else if (   p[2] == "sun")          resp = sunPosition();
                else if (   p[2] == "az")           resp = move(1, std::stoi(p[3]));
                else if (   p[2] == "el")           resp = move(0, std::stoi(p[3]));
                else if (   p[2] == "sel")          stopMoving(0);
                else if (   p[2] == "saz")          stopMoving(1);
                else if (CMD::mCmd.contains(p[2]))  resp = move(CMD::mCmd.find(p[2])->second);

                if (!resp.empty()) MqttClient::publish( resp );
            });
            }
        } 
    private:
        static void chaseTheSun() {
            if (instance_->moving[0] || instance_->moving[1]) return; // Don't chase while moving
            auto [azimuth, elevation] = Sun::getSunPosition();
            if (elevation < 4.0) {
                if (!instance_->parking) {
                    CMD::handleCommand("solar/cmd/el/0", "0"); // Move to parking position once
                    CMD::handleCommand("solar/cmd/az/180", "0"); 
                    std::cout << "[CMD] Sun below horizon, parking..." << std::endl;
                    instance_->parking = true;
                }
                return;
            }
            instance_->parking = false;
            elevation = 90.0 - elevation;
            elevation = elevation > CFG::elMaxDegrees ? CFG::elMaxDegrees : elevation;
            CMD::handleCommand("solar/cmd/el/" + std::to_string(static_cast<int>(elevation)), "0");
            CMD::handleCommand("solar/cmd/az/" + std::to_string(static_cast<int>(azimuth)), "0");
        }
        static inline std::unordered_map<std::string, std::string> mCmd = {
            {"mu", "ELCCW"}, {"md", "ELCW"}, {"me", "AZCCW"}, {"mw", "AZCW"} 
        };
        static inline std::unique_ptr<CMD> instance_ = nullptr; 
        bool tracking = false;
        bool parking = false;
        std::array<std::atomic<bool>, 2> moving = {false, false}; // moving[0]=El, moving[1]=Az
};
