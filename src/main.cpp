
#include "Sun.hpp"
#include "SensorController.h"
#include "Mqtt.hpp"
#include "SerialWorker.h"

int main() {
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

    



    while(true) {
        //AHRSPacket ahrs = SensorController::AHRS();
        //std::cout << "  Q:  " << ahrs.Qw  << " | " << ahrs.Qx  << " | " << ahrs.Qy  << " | " << ahrs.Qz  << " | " << std::endl;
        //std::cout << "  H:  " << ahrs.pitch  << " | " << ahrs.roll  << " | " << ahrs.yaw   << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
    return 0;
}
