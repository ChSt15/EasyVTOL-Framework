#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H



#include "stdint.h"
#include "math.h"



constexpr int64_t NANOSECONDS = 1;
constexpr int64_t MICROSECONDS = 1000*NANOSECONDS;
constexpr int64_t MILLISECONDS = 1000*MICROSECONDS;
constexpr int64_t SECONDS = 1000*MILLISECONDS;
constexpr int64_t MINUTES = 60*SECONDS;
constexpr int64_t HOURS = 60*MINUTES;
constexpr int64_t DAYS = 24*HOURS;
constexpr int64_t WEEKS = 7*DAYS;
constexpr int64_t MONTHS = 30*DAYS;
constexpr int64_t YEARS = 365*DAYS;



extern int64_t NOW();



#ifdef Arduino_h

inline int64_t NOW() {

    static uint32_t g_lastMicroseconds = 0;
    static int64_t g_currentTime = 0;

    int64_t dTime = micros()-g_lastMicroseconds;
    g_lastMicroseconds = micros();

    g_currentTime += dTime*MICROSECONDS;

    return g_currentTime;

}

#endif



#endif