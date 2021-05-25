#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H



#include "stdint.h"
#include "math.h"



#define SECONDS 1e9
#define MILLISECONDS 1e6
#define MICROSECONDS 1e3
#define NANOSECONDS 1



namespace {

    extern uint32_t g_lastMicroseconds;

    extern int64_t g_currentTime;

}



#ifdef Arduino_h

inline int64_t NOW() {

    int64_t dTime = micros()-g_lastMicroseconds;
    g_lastMicroseconds = micros();

    g_currentTime += dTime*MICROSECONDS;

    return g_currentTime;

}


#endif




#endif