#include <Arduino.h>

#include "definitions.h"

#include "threads/threads.h"

#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/air_data.h"

#include "outputs/rgb_led.h"


void setup() {

    Serial.begin(USB_BAUD_RATE);


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

    threadBegin();

}

void loop() {

    threadControl();

}