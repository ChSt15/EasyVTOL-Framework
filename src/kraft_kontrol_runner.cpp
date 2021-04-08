#include "kraft_kontrol_runner.h"



namespace KraftKontrolRunner {

    Vehicle* kraft = nullptr;
    ControlProfile* controlProfile = nullptr;

    Scheduler systemScheduler;

    void vehicleThread();
    void vehicleControlThread();

    void imuThread();
    void baroThread();

}



void KraftKontrolRunner::vehicleThread() {

    if (kraft != nullptr) kraft->thread(); //Make sure not to run a nullptr

}



void KraftKontrolRunner::vehicleControlThread() {

    if (controlProfile != nullptr) controlProfile->thread(); //Make sure not to run a nullptr

}



void KraftKontrolRunner::imuThread() {

    IMU.thread();

}



void KraftKontrolRunner::baroThread() {

    Baro.thread();

}



void KraftKontrolRunner::initialise() {


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

    systemScheduler.attachFunction(imuThread, 32000, TASK_PRIORITY::PRIORITY_REALTIME);
    systemScheduler.attachFunction(baroThread, Baro.get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_HIGH);
    //systemScheduler.attachFunction(LORA_2_4::deviceThread, 500, TASK_PRIORITY::PRIORITY_MIDDLE);
    //systemScheduler.attachFunction(GPS::deviceThread, 100, TASK_PRIORITY::PRIORITY_HIGH);
    //systemScheduler.attachFunction(RGBLED::deviceThread, 100, TASK_PRIORITY::PRIORITY_NONE);
    systemScheduler.attachFunction(IBUSReceiver::deviceThread, 1000, TASK_PRIORITY::PRIORITY_HIGH);

    systemScheduler.attachFunction(vehicleThread, kraft->getLoopRate_Hz(), TASK_PRIORITY::PRIORITY_REALTIME);

    systemScheduler.attachFunction(vehicleControlThread, controlProfile->get_LoopRate_Hz(), TASK_PRIORITY::PRIORITY_REALTIME);

}



void KraftKontrolRunner::loop() {

    systemScheduler.tick();

}


