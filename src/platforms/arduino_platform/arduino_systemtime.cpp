#include "KraftKontrol/platforms/arduino_platform/arduino_systemtime.h"


int64_t NOW() {

    static volatile uint32_t g_lastMicroseconds = 0;
    static volatile int64_t g_currentTime = 0;

    uint32_t time = micros();

    if (time != g_lastMicroseconds) {
        g_currentTime = g_currentTime + int64_t(time - g_lastMicroseconds)*MICROSECONDS;
        g_lastMicroseconds = time;
    }

    return g_currentTime;

}

