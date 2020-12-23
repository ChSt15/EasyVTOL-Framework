#ifndef CPU_LOAD_H
#define CPU_LOAD_H

#include "Arduino.h"

#include "TeensyThreads.h"


namespace CPULoad {

    namespace {

        volatile uint64_t loopCounter = 0;

        volatile uint32_t lastMeasure = 0;

        uint64_t calibrationCount = 1000;

        volatile float cpuLoad = 0.0f;

        int threadID = -1;

    }


    float getCPULoad() {
        return cpuLoad;
    }


    void loadThread() {

        Serial.println("Load thread Start!");

        while(true) {

            loopCounter++;

            if (micros() - lastMeasure >= 1000000L) {
                lastMeasure = micros();

                cpuLoad = (float)1.0f - loopCounter/calibrationCount;

                //cpuLoad = constrain(cpuLoad, 0.0f, 1.0f);

            }

        }

    }

    void begin() {

        loopCounter = 0; //Reset Loop counter
        lastMeasure = micros();

        threads.addThread(loadThread); //Start idle thread

        threads.delay(100);

        calibrationCount = loopCounter*100;

    }

}



#endif