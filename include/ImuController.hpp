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
#include <sstream>
#include <iomanip>
#include "Config.hpp"
#include "DBG.hpp"

struct VibrationStats {
    uint64_t acc_samples = 0;
    double acc_sum = 0.0;
    double acc_sq_sum = 0.0;
    float acc_min = 999.0f;
    float acc_max = 0.0f;
    float acc_max_dev = 0.0f;
    float acc_std = 0.0f;

    uint64_t gyro_samples = 0;
    double gyro_speed_sum = 0.0;     // deg/s
    double gyro_speed_sq_sum = 0.0;  // (deg/s)^2
    float gyro_max_speed = 0.0f;     // peak deg/s
    double gyro_angular_path = 0.0;  // total accumulated deg
    float gyro_rms = 0.0f;
    std::chrono::steady_clock::time_point last_gyro_time{};

    std::chrono::steady_clock::time_point window_start{};
    double duration_sec = 0.0;

    void reset() {
        acc_samples = 0;
        acc_sum = 0.0;
        acc_sq_sum = 0.0;
        acc_min = 999.0f;
        acc_max = 0.0f;
        acc_max_dev = 0.0f;
        acc_std = 0.0f;

        gyro_samples = 0;
        gyro_speed_sum = 0.0;
        gyro_speed_sq_sum = 0.0;
        gyro_max_speed = 0.0f;
        gyro_angular_path = 0.0;
        gyro_rms = 0.0f;
        last_gyro_time = std::chrono::steady_clock::time_point{};

        window_start = std::chrono::steady_clock::now();
        duration_sec = 0.0;
    }

    void finalize() {
        if (window_start.time_since_epoch().count() > 0) {
            auto now = std::chrono::steady_clock::now();
            duration_sec = std::chrono::duration<double>(now - window_start).count();
        }
        if (acc_samples > 1) {
            double mean_a = acc_sum / acc_samples;
            double var_a = (acc_sq_sum / acc_samples) - (mean_a * mean_a);
            acc_std = std::sqrt(std::max(0.0, var_a));
            acc_max_dev = std::max(std::abs(acc_max - mean_a), std::abs(acc_min - mean_a));

            if (acc_std < CFG::accStdNoiseThresholdG) {
                acc_std = 0.0f;
            }
            if (acc_max_dev < CFG::accMaxNoiseThresholdG) {
                acc_max_dev = 0.0f;
            }
        }
        if (gyro_samples > 0) {
            double var_g = gyro_speed_sq_sum / gyro_samples;
            gyro_rms = std::sqrt(std::max(0.0, var_g));
        }
    }

    std::string toJson() const {
        std::ostringstream ss;
        ss << std::fixed;
        ss << "{"
           << "\"samples\":" << acc_samples << ","
           << "\"duration_s\":" << std::setprecision(1) << duration_sec << ","
           << "\"acc_std_g\":" << std::setprecision(4) << acc_std << ","
           << "\"acc_max_g\":" << std::setprecision(3) << acc_max_dev << ","
           << "\"gyro_rms_dps\":" << std::setprecision(3) << gyro_rms << ","
           << "\"gyro_max_dps\":" << std::setprecision(2) << gyro_max_speed << ","
           << "\"gyro_deg\":" << std::setprecision(2) << gyro_angular_path
           << "}";
        return ss.str();
    }
};

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

    static void setVibrationSampling(bool enable) {
        if (!instance_) return;
        bool prev = instance_->sampling_enabled_.exchange(enable);
        if (!prev && enable) {
            std::lock_guard<std::mutex> lock(instance_->vibrationMutex_);
            instance_->vibrationStats_.reset();
        }
    }

    static bool isVibrationSampling() {
        if (!instance_) return false;
        return instance_->sampling_enabled_.load();
    }

    static std::string getAndResetVibrationReport() {
        if (!instance_) return "";
        std::lock_guard<std::mutex> lock(instance_->vibrationMutex_);
        if (instance_->vibrationStats_.acc_samples < 50) return "";
        instance_->vibrationStats_.finalize();
        std::string json = instance_->vibrationStats_.toJson();
        instance_->vibrationStats_.reset();
        return json;
    }

    static std::string getCurrentVibrationReport() {
        if (!instance_) return "{}";
        std::lock_guard<std::mutex> lock(instance_->vibrationMutex_);
        VibrationStats copy = instance_->vibrationStats_;
        copy.finalize();
        return copy.toJson();
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
                if (sampling_enabled_.load(std::memory_order_relaxed)) {
                    float gx = packet.rollSpeed;
                    float gy = packet.pitchSpeed;
                    float gz = packet.yawSpeed;
                    float gyro_mag = std::sqrt(gx * gx + gy * gy + gz * gz);
                    float gyro_dps = gyro_mag * (180.0f / static_cast<float>(M_PI));
                    float eff_gyro = (gyro_dps >= CFG::gyroNoiseDeadbandDps) ? gyro_dps : 0.0f;

                    auto now = std::chrono::steady_clock::now();
                    std::lock_guard<std::mutex> lock(vibrationMutex_);
                    if (vibrationStats_.last_gyro_time.time_since_epoch().count() > 0) {
                        double dt = std::chrono::duration<double>(now - vibrationStats_.last_gyro_time).count();
                        if (dt > 0.0 && dt < 0.5) {
                            vibrationStats_.gyro_angular_path += (eff_gyro * dt);
                        }
                    }
                    vibrationStats_.last_gyro_time = now;
                    vibrationStats_.gyro_samples++;
                    vibrationStats_.gyro_speed_sum += eff_gyro;
                    vibrationStats_.gyro_speed_sq_sum += (static_cast<double>(eff_gyro) * eff_gyro);
                    if (gyro_dps >= CFG::gyroNoiseDeadbandDps && gyro_dps > vibrationStats_.gyro_max_speed) {
                        vibrationStats_.gyro_max_speed = gyro_dps;
                    }
                }
            } else if (type == static_cast<uint8_t>(PacketType::IMU)) {
                IMUPacket packet;
                std::memcpy(&packet, buf.data() + 7, sizeof(IMUPacket));
                {
                    std::lock_guard<std::mutex> lock(imuMutex);
                    imu = packet;
                    lastImuTime_ = std::chrono::steady_clock::now();
                }
                if (sampling_enabled_.load(std::memory_order_relaxed)) {
                    float ax = packet.Accelerometer_X;
                    float ay = packet.Accelerometer_Y;
                    float az = packet.Accelerometer_Z;
                    float a_mag = std::sqrt(ax * ax + ay * ay + az * az);
                    if (a_mag > 4.0f) a_mag /= 9.80665f; // Normalize m/s^2 to g

                    std::lock_guard<std::mutex> lock(vibrationMutex_);
                    if (vibrationStats_.acc_samples == 0) {
                        vibrationStats_.window_start = std::chrono::steady_clock::now();
                    }
                    vibrationStats_.acc_samples++;
                    vibrationStats_.acc_sum += a_mag;
                    vibrationStats_.acc_sq_sum += (static_cast<double>(a_mag) * a_mag);
                    if (a_mag < vibrationStats_.acc_min) vibrationStats_.acc_min = a_mag;
                    if (a_mag > vibrationStats_.acc_max) vibrationStats_.acc_max = a_mag;
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

    std::atomic<bool> sampling_enabled_{false};
    std::mutex vibrationMutex_;
    VibrationStats vibrationStats_{};

    static inline std::unique_ptr<ImuController> instance_ = nullptr;
};

