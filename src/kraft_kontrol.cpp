#include "kraft_kontrol.h"



namespace KraftKontrol {

    Vehicle* kraft = nullptr;

    Scheduler systemScheduler;

    void vehicleThread();

}



void KraftKontrol::vehicleThread() {

    if (kraft != nullptr) kraft->thread(); //Make sure not to run a nullptr

}



void KraftKontrol::initialise() {

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


    systemScheduler.attachFunction(IMU::deviceThread, 32000, TASK_PRIORITY::PRIORITY_REALTIME);
    systemScheduler.attachFunction(AirData::deviceThread, 200, TASK_PRIORITY::PRIORITY_REALTIME);
    systemScheduler.attachFunction(LORA_2_4::deviceThread, 500, TASK_PRIORITY::PRIORITY_MIDDLE);
    systemScheduler.attachFunction(GPS::deviceThread, 100, TASK_PRIORITY::PRIORITY_HIGH);
    systemScheduler.attachFunction(RGBLED::deviceThread, 100, TASK_PRIORITY::PRIORITY_NONE);

    systemScheduler.attachFunction(vehicleThread, 8000, TASK_PRIORITY::PRIORITY_REALTIME);

}



void KraftKontrol::loop() {

    systemScheduler.tick();

}


