#pragma once
#include <wiringPi.h>
#include <atomic>
#include <functional>

const int P25 = 26;
const int P28 = 20; 
const int P29 = 21; 

const int CNTR = 17;

class GPIO {
public:
    explicit GPIO(bool pullUp = true)  :  ticks(0) {
        wiringPiSetupGpio();
        pinMode(P25, OUTPUT);
        pinMode(P28, OUTPUT);
        pinMode(CNTR, INPUT);
        if (pullUp) pullUpDnControl(CNTR, PUD_UP);

        wiringPiISR(CNTR, INT_EDGE_FALLING, &GPIO::handleISR);
        instance = this;
    }

    int getTicks() const {
        return ticks.load();
    }

    void reset() {
        ticks.store(0);
    }

private:
    int pin;
    std::atomic<int> ticks;

    static GPIO* instance;

    static void handleISR() {
        if (instance) instance->ticks++;
    }
};

// Define static member
GPIO* GPIO::instance = nullptr;