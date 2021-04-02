#include <Arduino.h>



#include "system_monitor.h"

#include "kraft_kontrol_runner.h"

#include "vehicle/starship/starship.h"
#include "flight_profiles/starship/starship_testing_profile.h"


Starship starship;
StarshipTestingProfile testingProfile;


ActuatorSetting actuator;


uint32_t frequency = 20;
uint32_t frequency2 = 20;


Scheduler music;

byte state = 0;


void beat() {

    static bool state = false;

    state = !state;

    actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPDR] = state*90*DEGREES;

    Serial.println(String("DR to ") + state*90);

}


void topfins() {

    static bool state = false;

    state = !state;

    actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPUR] = state*90*DEGREES;
    actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPUL] = state*90*DEGREES;

    Serial.println(String("UL and UR to ") + state*90);

}


void secondBeat() {

    static bool state = false;

    state = !state;

    actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPDL] = state*90*DEGREES;

    Serial.println(String("DL to ") + state*90);

}



void buzz() {

    actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCCW] = (0.5*sin((float)millis()/1000*2*3.1415*frequency) + 0.5)*0.02+0.005;

    Serial.println(String("BRRRRR at ") + frequency + "Hz");

}

void buzzSecond() {

    actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCW] = (0.5*sin((float)millis()/1000*2*3.1415*frequency2) + 0.5)*0.02+0.005;

    Serial.println(String("BRRRRR 2 at ") + frequency2 + "Hz");

}


void musicControl() {

    static uint32_t lastSwitch = 0;

    switch (state)
    {
    case 0:
        if (millis() - lastSwitch > 5000) {
            lastSwitch = millis();
            state = 1;
            music.attachFunction(beat, 1);
        }
        break;

    case 1:
        if (millis() - lastSwitch > 5000) {
            lastSwitch = millis();
            state = 2;
            music.attachFunction(secondBeat, 2);
        }
        break;

    case 2:
        if (millis() - lastSwitch > 5000) {
            lastSwitch = millis();
            state = 3;
            music.attachFunction(buzz, 1000);
            //actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCW] = 0.005;
        }
        break;

    case 3:
        if (millis() - lastSwitch > 2000) {
            lastSwitch = millis();
            state = 4;
            frequency = 80;
        }
        break;

    case 4:
        if (millis() - lastSwitch > 2000) {
            lastSwitch = millis();
            state = 5;
            music.detachFunction(buzz);
            music.detachFunction(beat);
            music.detachFunction(secondBeat);
            actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCCW] = 0;
        }
        break;  

    case 5:
        if (millis() - lastSwitch > 2000) {
            lastSwitch = millis();
            state = 6;
            //music.attachFunction(buzz);
            music.attachFunction(beat, 2);
            music.attachFunction(secondBeat, 4);
            music.attachFunction(topfins, 1);
            music.attachFunction(buzz, 1000);
            music.attachFunction(buzzSecond, 1000);
            frequency = 30;
            frequency = 32;
        }
        break; 

    case 6:
        if (millis() - lastSwitch > 5000) {
            lastSwitch = millis();
            state = 7;
            //music.attachFunction(buzz);
            music.detachFunction(beat);
            music.detachFunction(secondBeat);
            music.detachFunction(topfins);
            music.detachFunction(buzz);
            music.detachFunction(buzzSecond);
            actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCW] = 0.0;
            actuator.actuatorSetting[STARSHIP_ACTUATOR_MOTORCCW] = 0.0;
            actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPDL] = 90*DEGREES;
            actuator.actuatorSetting[STARSHIP_ACTUATOR_FLAPDR] = 90*DEGREES;
        }
        break; 


    default:
        break;
    }

    KraftKontrolRunner::kraft->getDynamicsPointer()->setActuatorsRawData(actuator);

}



void setup() {

    Serial.begin(115200);

    KraftKontrolRunner::controlProfile = &testingProfile;
    KraftKontrolRunner::kraft = &starship;

    KraftKontrolRunner::initialise();

    testingProfile.setVehiclePointer(&starship);

    delay(1000);

    //music.attachFunction(musicControl, 1000);
    //music.initialise();

}

void loop() {

    KraftKontrolRunner::loop();

    //music.tick();

    systemMonitor(); //Monitors data and sends it to usb serial port

}