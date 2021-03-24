#include "starship_dynamics.h"



void StarshipDynamics::thread() {

    if (!_interval.isTimeToRun()) return;


    static uint32_t lastswitch = 0;
    static float angle = 0;

    if (millis() - lastswitch >= 5000) {
        lastswitch = millis();

        if (angle == 90.0f*DEGREES) {
            angle = -90.0f*DEGREES;
        } else {
            angle = 90.0f*DEGREES;
        }

        flapULControl.setPosition(angle);
        flapURControl.setPosition(angle);

    }


    flapDLControl.thread();
    flapULControl.thread();
    flapDRControl.thread();
    flapURControl.thread();

    //Serial.println(String("Angle: ") + flapULControl.getPosition() + ", Speed: " + flapULControl.getVelocity());


}



void StarshipDynamics::init() {

    flapDLControl.setPosition(0);
    flapULControl.setPosition(0);
    flapDRControl.setPosition(0);
    flapURControl.setPosition(0);

}