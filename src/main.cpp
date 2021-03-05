#include <Arduino.h>



#include "kraft_kontrol.h"

#include "vehicle/starship/starship.h"


Starship starship;



void setup() {

    KraftKontrol::kraft = &starship;

    KraftKontrol::initialise();

}

void loop() {

    KraftKontrol::loop();

}