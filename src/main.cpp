
#include "Sun.hpp"
#include "SensorController.h"
#include "GPIO.hpp"

std::tm getUtcTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    return *gmtime(&tt); // UTC time
}


int main() {
    GPIO gpio; // wiringPi pin 0
    std::cout << "Ticks: " << std::endl;
    while (true) {
        int t = gpio.getTicks();
        std::cout << "Ticks: " << t << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }


    return 0;



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
