#include <Arduino.h>

#include "definitions.h"

#include "threads/threads.h"

#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/air_data.h"

#include "outputs/rgb_led.h"


void setup() {

    Serial.begin(USB_BAUD_RATE);

    Serial2.begin(9600);


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

    //threadBegin();

    //Serial.println("Thread start success: " + String(threadStartSuccess));

    /*pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);
    pinMode(4,OUTPUT);*/

}

void loop() {

    /*digitalWrite(11, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    digitalWrite(4, HIGH);
    delay(1000);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(4, LOW);
    delay(1000);*/
    //if (Serial.available()) Serial2.write(Serial.read());
    //if (Serial2.available()) Serial.write(Serial2.read());

    //IMU::deviceThread();

    //RGBLED::deviceThread();

    //AirData::deviceThread();

    

    //threadControl();

}