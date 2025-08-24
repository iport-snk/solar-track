#include "SerialWorker.h"
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

std::unique_ptr<SerialWorker> SerialWorker::instance_ = nullptr;

const auto ttl = std::chrono::milliseconds(1000);

void SerialWorker::init() {
    if (!instance_) {
        instance_ = std::make_unique<SerialWorker>();
    }
}

void SerialWorker::SEND(const std::string& command) {
    write(instance_->fd_, command.c_str(), command.size());
}

std::future<std::string> SerialWorker::CMD(const std::string& command) {
    if (!instance_) throw std::runtime_error("SerialWorker not initialized");
    return instance_->cmd(command);
}

SerialWorker::SerialWorker() {
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
    /*
    cleaner = setInterval([] {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        for (auto it = pending_.begin(); it != pending_.end(); ) {
            auto& [promise, expires_at] = it->second;
            if (now >= expires_at) {
                promise.set_exception(std::make_exception_ptr(std::runtime_error("Timeout")));
                it = pending_.erase(it);
            } else {
                ++it;
            }
        }

    }, 1000);
    */
    
}

SerialWorker::~SerialWorker() {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
    if (fd_ >= 0) close(fd_);
}

std::future<std::string> SerialWorker::cmd(const std::string& command) {
    std::promise<std::string> promise;
    auto future = promise.get_future();
    auto expires_at = std::chrono::steady_clock::now() + ttl;

    uint32_t ack_id = instance_->nextAckId();

    {
        std::lock_guard<std::mutex> lock(instance_->mutex_);
        instance_->pending_[ack_id] = std::make_tuple(std::move(promise), expires_at);
    }

    instance_->sendCommand(command, ack_id);
    return future;
}

void SerialWorker::sendCommand(const std::string& command, uint32_t ack_id) {
    //std::string frame = "\xAA" + oss.str();
    std::ostringstream oss;
    oss << "CMD:" << command << ':' << ack_id<< '\n';
    std::string frame = oss.str();

    //tcflush(instance_->fd_, TCOFLUSH);
    write(instance_->fd_, frame.c_str(), frame.size());
}

void SerialWorker::onAck(uint32_t ack_id, std::string data) {
    std::promise<std::string> promise;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pending_.find(ack_id);
        if (it == pending_.end()) return;

        promise = std::move(std::get<0>(it->second));
        pending_.erase(it);

    }
    promise.set_value(data);
}

void SerialWorker::ioLoop() {
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
            auto tokens = splitToVector(view , ':');

            if (tokens[0] == "ACK") {
                uint32_t id = std::strtoul(tokens[1].c_str(), nullptr, 10);
                onAck(id, tokens[2]);
            }
            
            syncing = false;
            index = 0;
        }
    }
}

uint32_t SerialWorker::nextAckId() {
    return ack_counter_++;
}


