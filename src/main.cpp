#include <Arduino.h>



#include "system_monitor.h"

#include "kraft_kontrol.h"

#include "vehicle/starship/starship.h"
#include "vehicle_control/manual/manual_profile.h"


Starship starship;
ManualControlProfile manualControl;



void setup() {

    Serial.begin(115200);

    KraftKontrol::kraft = &starship;
    KraftKontrol::controlProfile = &manualControl;
    //starship.getDynamicsPointer()->startActuatorTesting();

    KraftKontrol::initialise();

}

void loop() {

    KraftKontrol::loop();

    threadSystemMonitor(); //Monitors data and sends it to usb serial port

}