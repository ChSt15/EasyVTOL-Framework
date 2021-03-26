#include "starship_dynamics.h"



void StarshipDynamics::thread() {

    if (!_interval.isTimeToRun()) return;


    static uint32_t lastswitch = 0;
    static float angle = 0;



    if (!_actuatorManualMode && !_actuatorTesting) {   

        Vector direction = Quaternion(Vector(0,0,1), (float)millis()/1000).rotateVector(Vector(1,0,1));

        //calculate TVC angles
        float TVC1, TVC2, TVC3, TVC4;
        float twist = 0;
        _getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);

        _TVCServo1.setAngle(TVC1);
        _TVCServo2.setAngle(TVC2);
        _TVCServo3.setAngle(TVC3);
        _TVCServo4.setAngle(TVC4);

        _flapULControl.setPosition(90*DEGREES);
        _flapURControl.setPosition(90*DEGREES);
        _flapDRControl.setPosition(90*DEGREES);
        _flapDLControl.setPosition(90*DEGREES);

        _motorCW.setChannel(0);
        _motorCCW.setChannel(0);

    } else if (_actuatorManualMode && !_actuatorTesting) {

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

    } else if (_actuatorTesting && !_actuatorManualMode) { //Actuator testing mode

        if (millis() - lastswitch >= 5000) {
            lastswitch = millis();

            if (angle == 90.0f*DEGREES) {
                angle = 0.0f*DEGREES;
            } else {
                angle = 90.0f*DEGREES;
            }

        }

        //angle = 0;//(sin((float)millis()/3000)/2 + 0.5)*90*DEGREES;

        _flapULControl.setPosition(angle);
        _flapURControl.setPosition(angle);
        _flapDRControl.setPosition(angle);
        _flapDLControl.setPosition(angle);

        Vector direction = Quaternion(Vector(0,0,1), float(millis())/1000.0f).rotateVector(Vector(1,0,1));//_navigationData->attitude.copy().conjugate().rotateVector(Vector(0,0,1));

        //calculate TVC angles
        float TVC1, TVC2, TVC3, TVC4;
        float twist = 0;//45*DEGREES*sin((float)millis()/300);
        _getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);

        _TVCServo1.setAngle(TVC1);
        _TVCServo2.setAngle(TVC2);
        _TVCServo3.setAngle(TVC3);
        _TVCServo4.setAngle(TVC4);
        
    }

    _flapDLControl.thread();
    _flapULControl.thread();
    _flapDRControl.thread();
    _flapURControl.thread();

    _motorCW.setChannel(0);
    _motorCCW.setChannel(0);


}



void StarshipDynamics::_getTVCAngles(const Vector &direction, const float &twist, float &tvc1, float &tvc2, float &tvc3, float &tvc4) {

    tvc1 = atan2(direction.x, direction.z);
    tvc2 = atan2(direction.y, direction.z);
    tvc3 = tvc1 - twist;
    tvc4 = tvc2 - twist;

    tvc1 += twist;
    tvc2 += twist;

}



void StarshipDynamics::init() {

    _flapDLControl.setPosition(0);
    _flapULControl.setPosition(0);
    _flapDRControl.setPosition(0);
    _flapURControl.setPosition(0);

}