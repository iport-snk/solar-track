/*
    Z - yaw / heading / azimuth
    X - roll / rotating around X is rolling 
    Y - pitch / should be leveled to avoid tilt compensation in Mag and heading calculation
*/

#pragma once

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include "Config.hpp"

#pragma pack(push, 1)
struct IMUPacket {
    float Gyroscope_X;
    float Gyroscope_Y;
    float Gyroscope_Z;
    float Accelerometer_X;
    float Accelerometer_Y;
    float Accelerometer_Z;
    float mag_x;
    float mag_y;
    float mag_z;
    float IMU_Temperature;
    float Pressure;
    float Pressure_Temperature;
    int64_t timestamp;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AHRSPacket {
    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;
    float roll;
    float pitch;
    float yaw;
    float Qw;
    float Qx;
    float Qy;
    float Qz;
    int64_t timestamp;
};
#pragma pack(pop)

const uint8_t STF = 0xFC;
const uint8_t END = 0xFD;

enum class PacketType : uint8_t {
    IMU = 0x40,
    AHRS = 0x41
};

struct FrameHeader {
    uint8_t type;
    uint8_t len;
    uint8_t sn;
    uint8_t crc8;
    uint16_t crc16;
};

/**
 * 
 */
class ImuController {
public:
    static void init() {
        if (!instance_)  instance_ = std::make_unique<ImuController>();
    };
    static IMUPacket IMU();
    static AHRSPacket AHRS();
    static float getRoll() {
        std::lock_guard<std::mutex> lock(instance_->ahrsMutex);
        return (instance_->ahrs.roll * 180.0f / M_PI) * (CFG::invertRoll ? -1 : 1);
    };
    static float deltaEl(float elDegree);

    ImuController() {
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

        io_thread_ = std::thread(&ImuController::ioLoop, this);
    };
    ~ImuController() {
        running_ = false;
        if (io_thread_.joinable()) io_thread_.join();
        if (fd >= 0) close(fd);
    };

private:

    std::atomic<bool> running_  = true;
    std::thread io_thread_;

    void LaunchThread();

    void InitSerial();
    void ProcessPaket(PacketType packetType) {
        if (packetType == PacketType::AHRS) {
            std::lock_guard<std::mutex> lock(ahrsMutex);
            read(fd, &ahrs, sizeof(ahrs));
        } else if (packetType == PacketType::IMU) {
            std::lock_guard<std::mutex> lock(imuMutex);
            read(fd, &imu, sizeof(imu));
        }
    };
    

    IMUPacket imu;
    AHRSPacket ahrs;

    std::mutex imuMutex;
    std::mutex ahrsMutex;

    int fd = -1;  // Initialize to invalid fd

    void ioLoop() {
        uint8_t byte;
        uint8_t header[6]; // 0: type 1: len

        while (running_.load()) {  // Check running flag
            ssize_t bytes_read = read(fd, &byte, 1);
            if (bytes_read <= 0) {
                if (!running_.load()) break;  // Exit if stopping
                continue;  // Handle read errors gracefully
            }
            
            if (byte == STF) {
                read(fd, header, 6);
                ProcessPaket(static_cast<PacketType>(header[0]));
            } 
        }

    }

    
    static inline std::unique_ptr<ImuController> instance_ = nullptr;

};
