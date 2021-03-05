#include <Arduino.h>



#include "system_monitor.h"

#include "kraft_kontrol.h"

#include "vehicle/starship/starship.h"


Starship starship;



void setup() {

    Serial.begin(115200);

    KraftKontrol::kraft = &starship;

    KraftKontrol::initialise();

}

void loop() {

    KraftKontrol::loop();

    threadSystemMonitor(); //Monitors data and sends it to usb serial port

}