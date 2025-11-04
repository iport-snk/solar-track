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
class SensorController {
public:
    static void Start();
    static void Stop();
    static IMUPacket IMU();
    static AHRSPacket AHRS();
    static float getRoll();
    static float deltaEl(float elDegree);

private:
    static SensorController* Instance;

    std::atomic<bool> running;
    std::thread workerThread;

    void LaunchThread();

    void InitSerial();
    void ProcessPaket(PacketType packetType);

    IMUPacket imu;
    AHRSPacket ahrs;

    std::mutex imuMutex;
    std::mutex ahrsMutex;

    int fd = -1;  // Initialize to invalid fd

    SensorController();
    virtual ~SensorController();

};
