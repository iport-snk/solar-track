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
    void sendCommand(const std::string& command);
    void onAck(std::string ack_id, std::string data);


    std::future<std::string> cmd(const std::string& command);


    int fd_ = -1;
    std::thread io_thread_;
    std::unique_ptr<IntervalHandle> cleaner;
    std::atomic<bool> running_ = true;

    std::mutex mutex_;
    std::unordered_map<std::string, std::tuple< std::promise<std::string>, Clock::time_point>> pending_;

    static std::unique_ptr<SerialWorker> instance_;
};