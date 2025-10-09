#include <cmath>
#include "SensorController.h"
#include "Config.hpp"

SensorController* SensorController::Instance = nullptr;
SensorController::SensorController() {}
SensorController::~SensorController() {}

AHRSPacket SensorController::AHRS() {
    std::lock_guard<std::mutex> lock(Instance->ahrsMutex);
    return Instance->ahrs;
};

IMUPacket SensorController::IMU() {
    std::lock_guard<std::mutex> lock(Instance->imuMutex);
    return Instance->imu;
};

void SensorController::Start() {
    if (!Instance) {
        Instance = new SensorController();
        Instance->LaunchThread();
    }
};

void SensorController::InitSerial() {
    fd = open(CFG::ttyIMU, O_RDONLY | O_NOCTTY );
    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    // Raw binary mode
    options.c_cflag |= (CLOCAL | CREAD);              // Enable receiver, ignore modem control
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                           // 8 data bits
    options.c_cflag &= ~(PARENB | CSTOPB);            // No parity, 1 stop bit
    options.c_iflag &= ~(IXON | IXOFF | IXANY);       // No software flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST; 

    options.c_cc[VMIN] = 1;    // Wait for at least 1 byte
    options.c_cc[VTIME] = 0;   // Wait forever

    tcsetattr(fd, TCSANOW, &options);

};

float SensorController::deltaEl(float elDegree) {
    std::lock_guard<std::mutex> lock(Instance->ahrsMutex);

    float elDelta = elDegree ? (Instance->ahrs.roll  * (180.0 / M_PI) - elDegree) : 0 ;
    return std::abs(elDelta) > CFG::elThresholdDegrees ? elDelta : 0;
};

void SensorController::ProcessPaket(PacketType packetType) { 
    if (packetType == PacketType::AHRS) {
        std::lock_guard<std::mutex> lock(ahrsMutex);
        read(fd, &ahrs, sizeof(ahrs));
    } else if (packetType == PacketType::IMU) {
        std::lock_guard<std::mutex> lock(imuMutex);
        read(fd, &imu, sizeof(imu));
    }
};

void SensorController::LaunchThread() {
    running = true;
    std::thread([this]() {
        Instance->InitSerial();

        if (fd < 0) {
            perror("Failed to open serial port");
            return;
        }

        uint8_t byte;
        uint8_t header[6]; // 0: type 1: len

        while (true) {
            read(fd, &byte, 1);
            if (byte == STF) {
                read(fd, header, 6);
                ProcessPaket(static_cast<PacketType>(header[0]));
            } 
        }

        close(fd);
    }).detach();
}
