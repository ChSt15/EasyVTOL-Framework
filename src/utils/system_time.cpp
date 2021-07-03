#include "KraftKontrol/utils/system_time.h"



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



