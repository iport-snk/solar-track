
#include "Sun.hpp"
#include "SensorController.h"
#include <wiringPi.h>

const int RELAY_A = 0; // wiringPi pin 0 = BCM GPIO 17
const int RELAY_B = 2; // wiringPi pin 2 = BCM GPIO 27


std::tm getUtcTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    return *gmtime(&tt); // UTC time
}


int main() {
    wiringPiSetupGpio();
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);
    sleep(2);
    digitalWrite(17, LOW);



    SensorController::Start();

    auto [azimuth, elevation] = Sun::getSunPosition();
    std::cout << "Sun : " << azimuth << " |  " << elevation << std::endl;

    



    while(true) {
        AHRSPacket ahrs = SensorController::AHRS();
        std::cout << "  Q:  " << ahrs.Qw  << " | " << ahrs.Qx  << " | " << ahrs.Qy  << " | " << ahrs.Qz  << " | " << std::endl;
        std::cout << "  H:  " << ahrs.pitch  << " | " << ahrs.roll  << " | " << ahrs.yaw   << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    
    return 0;
}
