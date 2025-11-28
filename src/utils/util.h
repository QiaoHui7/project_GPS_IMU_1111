#ifndef UTIL_H
#define UTIL_H

#include <cstdint>
#include <termios.h>

struct SENSOR_TIME_S {
    uint64_t ts_us;
    int hour;
    int min;
    int sec;
    int ms;
};

int open_serial_port(const char* device, speed_t baud_rate);

#endif
