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
#include <algorithm>
#include <limits>
#include <cmath>
#include "Config.hpp"
#include "DBG.hpp"

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

class ImuController {
public:
    static void init() {
        if (!instance_) instance_ = std::make_unique<ImuController>();
    }

    static float getRoll() {
        if (!instance_) return std::numeric_limits<float>::quiet_NaN();
        std::lock_guard<std::mutex> lock(instance_->ahrsMutex);
        auto now = std::chrono::steady_clock::now();
        if (now - instance_->lastAhrsTime_ > std::chrono::seconds(2)) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        float deg = (instance_->ahrs.roll * 180.0f / M_PI) * (CFG::invertRoll ? -1.0f : 1.0f);
        if (std::isnan(deg) || std::isinf(deg)) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return deg;
    }

    static bool isHealthy() {
        if (!instance_) return false;
        std::lock_guard<std::mutex> lock(instance_->ahrsMutex);
        auto now = std::chrono::steady_clock::now();
        return (now - instance_->lastAhrsTime_ <= std::chrono::seconds(2));
    }

    ImuController() {
        openPort();
        io_thread_ = std::thread(&ImuController::ioLoop, this);
    }

    ~ImuController() {
        running_ = false;
        if (io_thread_.joinable()) io_thread_.join();
        closePort();
    }

private:
    void openPort() {
        closePort();
        fd = open(CFG::ttyIMU, O_RDONLY | O_NOCTTY);
        if (fd < 0) return;

        struct termios options{};
        if (tcgetattr(fd, &options) != 0) {
            closePort();
            return;
        }

        cfsetispeed(&options, B921600);
        cfsetospeed(&options, B921600);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~(PARENB | CSTOPB);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 0;   // Non-blocking / timeout read
        options.c_cc[VTIME] = 2;  // 200 ms timeout

        tcsetattr(fd, TCSANOW, &options);
    }

    void closePort() {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

    void reconnect() {
        closePort();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        openPort();
        if (fd >= 0) {
            DBG::log("[IMU] Reconnected successfully to ", CFG::ttyIMU);
        }
    }

    void processBuffer(std::vector<uint8_t>& buf) {
        while (buf.size() >= 8) {
            if (buf[0] != STF) {
                auto it = std::find(buf.begin(), buf.end(), STF);
                if (it == buf.end()) {
                    buf.clear();
                    break;
                }
                buf.erase(buf.begin(), it);
                if (buf.size() < 8) break;
            }

            uint8_t type = buf[1];
            uint8_t len = buf[2];

            if (type == static_cast<uint8_t>(PacketType::AHRS)) {
                if (len != sizeof(AHRSPacket)) {
                    buf.erase(buf.begin());
                    continue;
                }
            } else if (type == static_cast<uint8_t>(PacketType::IMU)) {
                if (len != sizeof(IMUPacket)) {
                    buf.erase(buf.begin());
                    continue;
                }
            } else {
                buf.erase(buf.begin());
                continue;
            }

            size_t totalFrameSize = 1 + 6 + len + 1;
            if (buf.size() < totalFrameSize) {
                break; // Incomplete packet, wait for more data
            }

            if (buf[totalFrameSize - 1] != END) {
                buf.erase(buf.begin());
                continue;
            }

            // Valid packet extracted
            if (type == static_cast<uint8_t>(PacketType::AHRS)) {
                AHRSPacket packet;
                std::memcpy(&packet, buf.data() + 7, sizeof(AHRSPacket));
                {
                    std::lock_guard<std::mutex> lock(ahrsMutex);
                    ahrs = packet;
                    lastAhrsTime_ = std::chrono::steady_clock::now();
                }
            } else if (type == static_cast<uint8_t>(PacketType::IMU)) {
                IMUPacket packet;
                std::memcpy(&packet, buf.data() + 7, sizeof(IMUPacket));
                {
                    std::lock_guard<std::mutex> lock(imuMutex);
                    imu = packet;
                    lastImuTime_ = std::chrono::steady_clock::now();
                }
            }

            lastPacketTime_ = std::chrono::steady_clock::now();
            buf.erase(buf.begin(), buf.begin() + totalFrameSize);
        }
    }

    void ioLoop() {
        std::vector<uint8_t> rxBuffer;
        rxBuffer.reserve(512);

        while (running_.load()) {
            if (fd < 0) {
                reconnect();
                if (fd < 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }
            }

            uint8_t temp[128];
            ssize_t bytes_read = read(fd, temp, sizeof(temp));

            if (bytes_read < 0) {
                // Read error (USB disconnected, EIO, etc.)
                DBG::log("[IMU] Read error on serial port, reconnecting...");
                reconnect();
                rxBuffer.clear();
                continue;
            }

            if (bytes_read > 0) {
                rxBuffer.insert(rxBuffer.end(), temp, temp + bytes_read);
                processBuffer(rxBuffer);
            } else {
                // Timeout (bytes_read == 0 with VMIN=0, VTIME=2)
                auto now = std::chrono::steady_clock::now();
                if (now - lastPacketTime_ > std::chrono::seconds(2)) {
                    DBG::log("[IMU] Data timeout (>2s), reconnecting serial port...");
                    reconnect();
                    rxBuffer.clear();
                }
            }

            if (rxBuffer.size() > 2048) {
                rxBuffer.clear();
            }
        }
    }

    std::atomic<bool> running_{true};
    std::thread io_thread_;

    IMUPacket imu{};
    AHRSPacket ahrs{};

    std::mutex imuMutex;
    std::mutex ahrsMutex;

    std::chrono::steady_clock::time_point lastAhrsTime_{};
    std::chrono::steady_clock::time_point lastImuTime_{};
    std::chrono::steady_clock::time_point lastPacketTime_{};

    int fd = -1;

    static inline std::unique_ptr<ImuController> instance_ = nullptr;
};

