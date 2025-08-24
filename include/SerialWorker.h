#pragma once
#include <future>
#include <array>
#include <unordered_map>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include "IntervalRunner.hpp"

using Clock = std::chrono::steady_clock;

class SerialWorker {
public:
    static void init();  // Explicit setup
    static std::future<std::string> CMD(const std::string& command);
    static void SEND(const std::string& msg);
    SerialWorker();
    ~SerialWorker();

private:
    // std::vector<std::string> splitToVector(const char* buffer, char delimiter);
    
    void ioLoop();
    void sendCommand(const std::string& command, uint32_t ack_id);
    void onAck(uint32_t ack_id, std::string data);
    uint32_t nextAckId();

    std::future<std::string> cmd(const std::string& command);


    int fd_ = -1;
    std::thread io_thread_;
    std::unique_ptr<IntervalHandle> cleaner;
    std::atomic<bool> running_ = true;

    std::mutex mutex_;
    std::unordered_map<uint32_t, std::tuple< std::promise<std::string>, Clock::time_point>> pending_;
    uint32_t ack_counter_ = 1;

    static std::unique_ptr<SerialWorker> instance_;
};