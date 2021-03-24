#include "starship_dynamics.h"



void StarshipDynamics::thread() {

    if (!_interval.isTimeToRun()) return;


    static uint32_t lastswitch = 0;
    static float angle = 0;





    if (millis() - lastswitch >= 5000) {
        lastswitch = millis();

        if (angle == 90.0f*DEGREES) {
            angle = 0.0f*DEGREES;
        } else {
            angle = 90.0f*DEGREES;
        }

        if (_actuatorTesting && !_actuatorManualMode) {
            _flapULControl.setPosition(angle);
            _flapURControl.setPosition(angle);
            _flapDRControl.setPosition(angle);
            _flapDLControl.setPosition(angle);
            _TVCServo1.setChannel(angle/(90*DEGREES));
            _TVCServo2.setChannel(angle/(90*DEGREES));
            _TVCServo3.setChannel(angle/(90*DEGREES));
            _TVCServo4.setChannel(angle/(90*DEGREES));
        }

    }

    if (_actuatorManualMode) {
        _flapULControl.setPosition(_actuatorManualSetpoint.actuatorSetting[0]);
        _flapURControl.setPosition(_actuatorManualSetpoint.actuatorSetting[1]);
        _flapDRControl.setPosition(_actuatorManualSetpoint.actuatorSetting[2]);
        _flapDLControl.setPosition(_actuatorManualSetpoint.actuatorSetting[3]);
        _TVCServo1.setChannel(_actuatorManualSetpoint.actuatorSetting[4]);
        _TVCServo2.setChannel(_actuatorManualSetpoint.actuatorSetting[5]);
        _TVCServo3.setChannel(_actuatorManualSetpoint.actuatorSetting[6]);
        _TVCServo4.setChannel(_actuatorManualSetpoint.actuatorSetting[7]);
        _motorCW.setChannel(_actuatorManualSetpoint.actuatorSetting[8]);
        _motorCCW.setChannel(_actuatorManualSetpoint.actuatorSetting[9]);
    }


    _flapDLControl.thread();
    _flapULControl.thread();
    _flapDRControl.thread();
    _flapURControl.thread();

    Serial.println(String("Angle UL: ") + _flapULControl.getPosition() + ", Speed: " + _flapULControl.getVelocity());


}



void StarshipDynamics::init() {

    _flapDLControl.setPosition(0);
    _flapULControl.setPosition(0);
    _flapDRControl.setPosition(0);
    _flapURControl.setPosition(0);

}