
#include "Sun.hpp"
#include "SensorController.h"
#include "Mqtt.hpp"
#include "SerialWorker.h"
#include "Globals.hpp"
#include <signal.h>

volatile bool keep_running = true;

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    SerialWorker::SEND("ELSTOP\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    SerialWorker::SEND("AZSTOP\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    keep_running = false;
}

int main() {
    // Register signal handlers for graceful shutdown
    signal(SIGINT, signal_handler);   // Ctrl+C
    signal(SIGTERM, signal_handler);  // Termination request
    SerialWorker::init();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


    // TODO: need to go through check list ensuring Arduino functioning well
    // do the same with N100
    auto future = SerialWorker::CMD("PING");
    std::string response = future.get();
    std::cout << response << std::endl;


    SensorController::Start();
    MqttClient::init();

    //
    // =========== END of tests
    

  

    auto [azimuth, elevation] = Sun::getSunPosition();
    std::cout << "Sun : " << azimuth << " |  " << elevation << std::endl;

    


    while(keep_running) {
        #ifndef NDEBUG
        //float rollDegrees = SensorController::getRoll();
        //std::cout << "  Roll:  " << static_cast<int>(rollDegrees) << "°" << std::endl;
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        #endif
    }
    
    // Cleanup before exit
    SensorController::Stop();
    return 0;
}
