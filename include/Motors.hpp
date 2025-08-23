#pragma once
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <sys/ioctl.h>   // ioctl()
#include <SerialWorker.h>
#include "Config.hpp"

class Motors {
public:
    static bool init() {
        Motors& m = instance();
        sleep(1);
        if (m.fd_ < 0) return false;
tcflush(m.fd_, TCIFLUSH);
        // Send handshake
        const char* ping = "ping\n";
//int flags = TIOCM_DTR | TIOCM_RTS;
//ioctl(m.fd_, TIOCMBIS, &flags);  // Set DTR and RTS
//tcflush(m.fd_, TCIOFLUSH);
        ssize_t nnn = write(m.fd_, ping, strlen(ping));

tcdrain(m.fd_);
tcflush(m.fd_, TCIOFLUSH);

//sleep(1); 
        
        // Wait for response
        char buf[64] = {};
        int n = read(m.fd_, buf, sizeof(buf) - 1);
        
        //sleep(2);

        if (n <= 0 || strncmp(buf, "OK", 2) != 0) {
            fprintf(stderr, "[Motors] Arduino not responding. Exiting.\n");
            exit(1);
        }

        return true;
    }

    static Motors& instance() {
        static Motors motors;
        return motors;
    }

    void send(const char* cmd) {
        if (fd_ >= 0) write(fd_, cmd, strlen(cmd));
    }

    static void moveU() {}
    static void moveD() {}
    static void moveE() {}
    static void moveW() {}
    static void stopEl() {}
    static void stopAz() {}
    static void relays() {
        auto future = SerialWorker::CMD("PING");
        std::string response = future.get();
        std::cout <<  response << std::endl;
    }
    static bool isAzMov;
    static bool isElMov;

private:
    Motors() {
        fd_ = open(CFG::ttyMotors, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            fprintf(stderr, "[Motors] Failed to open serial port.\n");
            return;
        }

        struct termios tty {};
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            fd_ = -1;
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 2;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(fd_, TCSANOW, &tty);
        tcflush(fd_, TCIFLUSH);

    }

    ~Motors() {
        if (fd_ >= 0) close(fd_);
    }

    int fd_ = -1;
};

bool Motors::isAzMov = false;
bool Motors::isElMov = false;
