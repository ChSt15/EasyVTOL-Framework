#include "kraft_kontrol.h"



namespace KraftKontrol {

    Vehicle* kraft = nullptr;
    ControlProfile* controlProfile = nullptr;

    Scheduler systemScheduler;

    void vehicleThread();
    void vehicleControlThread();

}



void KraftKontrol::vehicleThread() {

    if (kraft != nullptr) kraft->thread(); //Make sure not to run a nullptr

}



void KraftKontrol::vehicleControlThread() {

    if (controlProfile != nullptr) controlProfile->thread(); //Make sure not to run a nullptr

}



void KraftKontrol::initialise() {


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


    controlProfile->setVehiclePointer(kraft);


    systemScheduler.attachFunction(IMU::deviceThread, 32000, TASK_PRIORITY::PRIORITY_REALTIME);
    systemScheduler.attachFunction(AirData::deviceThread, 200, TASK_PRIORITY::PRIORITY_HIGH);
    systemScheduler.attachFunction(LORA_2_4::deviceThread, 500, TASK_PRIORITY::PRIORITY_MIDDLE);
    systemScheduler.attachFunction(GPS::deviceThread, 100, TASK_PRIORITY::PRIORITY_HIGH);
    systemScheduler.attachFunction(RGBLED::deviceThread, 100, TASK_PRIORITY::PRIORITY_NONE);
    systemScheduler.attachFunction(IBUSReceiver::deviceThread, 1000, TASK_PRIORITY::PRIORITY_HIGH);

    systemScheduler.attachFunction(vehicleThread, kraft->getLoopRate_Hz(), TASK_PRIORITY::PRIORITY_REALTIME);

    systemScheduler.attachFunction(vehicleControlThread, controlProfile->getLoopRate_Hz(), TASK_PRIORITY::PRIORITY_REALTIME);

}



void KraftKontrol::loop() {

    systemScheduler.tick();

}


