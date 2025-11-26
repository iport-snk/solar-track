#pragma once
#include <future>
#include <array>
#include <unordered_map>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <ranges>
#include "Config.hpp"

using Clock = std::chrono::steady_clock;

class SerialWorker {
public:
    static void init() {
        if (!instance_) instance_ = std::make_unique<SerialWorker>();

    };
    //static std::future<std::string> CMD(const std::string& command) {
    //    if (!instance_) throw std::runtime_error("SerialWorker not initialized");
    //    return instance_->cmd(command);
    //};
    static void SEND(const std::string& msg) {
        write(instance_->fd_, msg.c_str(), msg.size());
    };
    SerialWorker() {
        fd_ = open(CFG::ttyMotors, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) throw std::runtime_error("Failed to open TTY");

        termios tty{};
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 128;
        tty.c_cc[VTIME] = 1;
        tcsetattr(fd_, TCSANOW, &tty);

        io_thread_ = std::thread(&SerialWorker::ioLoop, this);
    };
    ~SerialWorker() {
        running_ = false;
        if (io_thread_.joinable()) io_thread_.join();
        if (fd_ >= 0) close(fd_);
    };
    static std::string cmd(const std::string &command) {
        auto future = instance_->cmd_(command);
        std::string state = future.get();
        return state;
    }

private:
    // std::vector<std::string> splitToVector(const char* buffer, char delimiter);
    void ioLoop() {
        char buffer[128];  // generous buffer for overflow safety
        size_t index = 0;
        bool syncing = false;

        while (running_) {
            char buffer[128];  // generous buffer for overflow safety
            char byte;
            ssize_t n = read(fd_, &byte, 1);
            //if (n != 1) continue;

            if (!syncing) {
                if (byte == 0xAA) syncing = true;
                continue;
            }

            buffer[index++] = byte;

            if (byte == '\n' || byte == '\r' || index >= sizeof(buffer)) {
                std::string_view view(buffer, index - 1);
                // std::cout << "Received: " << view << std::endl;
                auto tokens = splitToVector<std::string>(view , ':');

                if (tokens[0] == "ACK") {
                    onAck(tokens[1], tokens[2]);
                }
                
                syncing = false;
                index = 0;
            }
        }
    };
    void sendCommand(const std::string& command) {
        //std::string frame = "\xAA" + oss.str();
        std::ostringstream oss;
        oss << "CMD:" << command << '\n';
        std::string frame = oss.str();

        //tcflush(instance_->fd_, TCOFLUSH);
        write(instance_->fd_, frame.c_str(), frame.size());
    };
    void onAck(std::string ack_id, std::string data) {
        std::promise<std::string> promise;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = pending_.find(ack_id);
            if (it == pending_.end()) return;

            promise = std::move(std::get<0>(it->second));
            pending_.erase(it);

        }
        promise.set_value(data);
    };


    std::future<std::string> cmd_(const std::string& command) {
        const auto ttl = std::chrono::milliseconds(1000);
        std::promise<std::string> promise;
        auto future = promise.get_future();
        auto expires_at = std::chrono::steady_clock::now() + ttl;

        {
            std::lock_guard<std::mutex> lock(instance_->mutex_);
            auto key = command.substr(0, command.find(':'));
            instance_->pending_[key] = std::make_tuple(std::move(promise), expires_at);
        }

        instance_->sendCommand(command);
        return future;
    };


    int fd_ = -1;
    std::thread io_thread_;

    std::atomic<bool> running_ = true;

    std::mutex mutex_;
    std::unordered_map<std::string, std::tuple< std::promise<std::string>, Clock::time_point>> pending_;

    static inline std::unique_ptr<SerialWorker> instance_ = nullptr;
};
