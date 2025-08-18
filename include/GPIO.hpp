#pragma once
#include <wiringPi.h>
#include <atomic>
#include <functional>

const int P25 = 26;
const int P28 = 20; 
const int P29 = 21; 
const int P27 = 16; 

const int CNTR = 17;

class GPIO {
public:
    explicit GPIO(bool pullUp = true)  :  ticks(0) {
        wiringPiSetupGpio();
        pinMode(P25, OUTPUT);
        digitalWrite(P25, HIGH);
        pinMode(P28, OUTPUT);
        digitalWrite(P28, HIGH);
        pinMode(P29, OUTPUT);
        digitalWrite(P27, HIGH);
        pinMode(P27, OUTPUT);
        digitalWrite(P29, HIGH);
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
    static void stopEl()        { digitalWrite(P25, HIGH);      digitalWrite(P28, HIGH);    };
    static void moveU()         { digitalWrite(P25, HIGH);      digitalWrite(P28, LOW);     };
    static void moveD()         { digitalWrite(P25, LOW);       digitalWrite(P28, HIGH);    };

    static void stopAz()        { digitalWrite(P29, HIGH);      digitalWrite(P27, HIGH);    };
    static void moveE()         { digitalWrite(P29, HIGH);      digitalWrite(P27, LOW);     };
    static void moveW()         { digitalWrite(P29, LOW);       digitalWrite(P27, HIGH);    };



    static void startMovement(float az, float el) {

        // AHRSPacket ahrs = SensorController::AHRS();


        /*
        Motor::azimuth(deltaSign(targetAz - currentAz));
        Motor::elevation(deltaSign(targetEl - currentEl));

        while (!isTargetReached(targetAz, targetEl)) {
            currentAz = IMU::azimuth();
            currentEl = IMU::elevation();

            Motor::azimuth(deltaSign(targetAz - currentAz));
            Motor::elevation(deltaSign(targetEl - currentEl));

            delay(50);
        }

        Motor::stopAll();
        TrackerFSM::onTargetReached();
        */
       // std::cout << ahrs.roll << std::endl;
        
    }


private:
    int pin;
    std::atomic<int> ticks;
    

    static GPIO* instance;

    static void handleISR() {
        if (instance) instance->ticks++;
    }

    /*
    static bool isTargetReached(double az, double el) {
        return std::abs(IMU::azimuth() - az) < azThreshold &&
               std::abs(IMU::elevation() - el) < elThreshold;
    }

    static Direction deltaSign(double delta) {
        if (std::abs(delta) < threshold) return Direction::STOP;
        return delta > 0 ? Direction::CW : Direction::CCW;
    }
    */
    

};

// Define static member
GPIO* GPIO::instance = nullptr;