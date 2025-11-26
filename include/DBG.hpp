#pragma once
#include <iostream>
#include <mutex>
#include <sstream>
class DBG {
public:
    template<typename... Args>
    static void log(Args&&... args) {
        std::lock_guard<std::mutex> lock(mtx);
        (std::cout << ... << args) << std::endl;
    }
private:
    inline static std::mutex mtx;
};