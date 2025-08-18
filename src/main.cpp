
#include "Sun.hpp"
#include "SensorController.h"
#include "GPIO.hpp"
#include "Mqtt.hpp"

int main() {
    SensorController::Start();
    MqttClient::init("tcp://broker.hivemq.com", 1883);
    



    GPIO gpio; // wiringPi pin 0


    

    auto [azimuth, elevation] = Sun::getSunPosition();
    std::cout << "Sun : " << azimuth << " |  " << elevation << std::endl;

    



    while(true) {
        //AHRSPacket ahrs = SensorController::AHRS();
        //std::cout << "  Q:  " << ahrs.Qw  << " | " << ahrs.Qx  << " | " << ahrs.Qy  << " | " << ahrs.Qz  << " | " << std::endl;
        //std::cout << "  H:  " << ahrs.pitch  << " | " << ahrs.roll  << " | " << ahrs.yaw   << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
    return 0;
}
