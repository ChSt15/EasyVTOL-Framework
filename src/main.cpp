#include <Arduino.h>

#include "definitions.h"

#include "threads/threads.h"



void setup() {

    Serial.begin(115200);


    #ifdef WAIT_FOR_SERIAL
        Serial.println("Waiting for Serial comms");
        //Wait for Serial message to continue
        while (!Serial.available()) {
            delay(100);
        }
        Serial.flush();
    #endif
    

    Serial.println("Serial connection started.");
    Serial.println("CPU Speed: " + String(F_CPU_ACTUAL));
    delay(500);

    #ifndef DISABLE_MULTITHREADING
        threadBegin();
    #endif

}

void loop() {

    threadControl();

}