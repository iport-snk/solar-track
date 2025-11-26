extern "C" {
#include <MQTTClient.h>
}
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <functional>
#include "SerialWorker.h"
#include "Mqtt.hpp"
#include "Cmd.hpp"
#include "Config.hpp"
#include "ImuController.hpp"
#include "Cmd.hpp"

int main() { 
    CFG::init();
    SerialWorker::init();
    ImuController::init();
    MqttClient::init([](const std::string_view& topic, const std::string_view& payload) {
        CMD::handleCommand(std::string(topic), std::string(payload));
        std::cout << "[CALLBACK] Topic: " << std::string(topic) << ", Payload: " << std::string(payload) << std::endl;
    });
    CMD::init();
    while (true) {
        MqttClient::loop(); 
        CMD::loop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    MqttClient::shutdown();
    return 0;

}