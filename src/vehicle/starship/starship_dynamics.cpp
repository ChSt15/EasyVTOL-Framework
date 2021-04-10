#include "starship_dynamics.h"



void StarshipDynamics::thread() {

    if (!_interval.isTimeToRun()) return;


    static uint32_t lastswitch = 0;
    static float angle = 0;

    

    if (!_actuatorManualMode && !_actuatorTesting) {   

        float force = 0;
        Vector directionBuf = Vector(0,0,1);

        _TVCCalculator.dynamicsSetpoint(*_dynamicSetpoint);
        _TVCCalculator.getTVCSettings(force, directionBuf);

        Vector direction;
        direction.z = directionBuf.z;
        direction.x = -directionBuf.y;
        direction.y = directionBuf.x;

        //calculate TVC angles
        float TVC1, TVC2, TVC3, TVC4;
        float twist = -constrain(_dynamicSetpoint->torqe.z, -45*DEGREES, 45*DEGREES);
        _getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);


        _TVCServo1.setAngle(TVC1);
        _TVCServo2.setAngle(TVC2);
        _TVCServo3.setAngle(TVC3);
        _TVCServo4.setAngle(TVC4);

        _flapULControl.setPosition(0*DEGREES);
        _flapURControl.setPosition(0*DEGREES);
        _flapDRControl.setPosition(0*DEGREES);
        _flapDLControl.setPosition(0*DEGREES);

        _motorCW.setChannel(force/MAX_TVC_FORCE);
        _motorCCW.setChannel(force/MAX_TVC_FORCE);

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

        if (millis() - lastswitch >= 1000) {
            lastswitch = millis();

            if (angle == 90.0f*DEGREES) {
                angle = 0.0f*DEGREES;
            } else {
                angle = 90.0f*DEGREES;
                _flapTestCounter++;
            }

        }

        switch (_flapTestStage) {

        case FLAP_TEST_STAGE::TOP_LEFT:
            _flapULControl.setPosition(angle);
            _flapURControl.setPosition(0);
            _flapDRControl.setPosition(0);
            _flapDLControl.setPosition(0);
            if (_flapTestCounter > 1) {
                _flapTestCounter = 0;
                _flapTestStage = FLAP_TEST_STAGE::TOP_RIGHT;
            }
            break;

        case FLAP_TEST_STAGE::TOP_RIGHT:
            _flapULControl.setPosition(0);
            _flapURControl.setPosition(angle);
            _flapDRControl.setPosition(0);
            _flapDLControl.setPosition(0);
            if (_flapTestCounter > 2) {
                _flapTestCounter = 0;
                _flapTestStage = FLAP_TEST_STAGE::BOTTOM_LEFT;
            }
            break;

        case FLAP_TEST_STAGE::BOTTOM_LEFT:
            _flapULControl.setPosition(0);
            _flapURControl.setPosition(0);
            _flapDRControl.setPosition(0);
            _flapDLControl.setPosition(angle);
            if (_flapTestCounter > 3) {
                _flapTestCounter = 0;
                _flapTestStage = FLAP_TEST_STAGE::BOTTOM_RIGHT;
            }
            break;

        case FLAP_TEST_STAGE::BOTTOM_RIGHT:
            _flapULControl.setPosition(0);
            _flapURControl.setPosition(0);
            _flapDRControl.setPosition(angle);
            _flapDLControl.setPosition(0);
            if (_flapTestCounter > 4) {
                _flapTestCounter = 0;
                _flapTestStage = FLAP_TEST_STAGE::TOP_LEFT;
            }
            break;
        
        default:
            break;
        }

        Vector direction = Quaternion(Vector(0,0,1), float(millis())/1000.0f).rotateVector(Vector(1,0,1));//_navigationData->attitude.copy().conjugate().rotateVector(Vector(0,0,1));

        //calculate TVC angles
        float TVC1, TVC2, TVC3, TVC4;
        float twist = 45*DEGREES*sin((float)millis()/300);
        _getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);

        _TVCServo1.setAngle(TVC1);
        _TVCServo2.setAngle(TVC2);
        _TVCServo3.setAngle(TVC3);
        _TVCServo4.setAngle(TVC4);

        _motorCW.setChannel(0);
        _motorCCW.setChannel(0);
        
    }

    _flapDLControl.thread();
    _flapULControl.thread();
    _flapDRControl.thread();
    _flapURControl.thread();

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

    _flapDLControl.setPosition(90*DEGREES);
    _flapULControl.setPosition(90*DEGREES);
    _flapDRControl.setPosition(90*DEGREES);
    _flapURControl.setPosition(90*DEGREES);

    _TVCServo1.activateChannel();
    _TVCServo2.activateChannel();
    _TVCServo3.activateChannel();
    _TVCServo4.activateChannel();
    _flapDL.activateChannel();
    _flapUL.activateChannel();
    _flapDR.activateChannel();
    _flapUR.activateChannel();

    _motorCW.activateChannel();
    _motorCCW.activateChannel();

    _motorCW.setChannel(0);
    _motorCCW.setChannel(0);

    _TVCCalculator.setDynamicConstraints(MAX_TVC_FORCE, MAX_TVC_ANGLE);
    _TVCCalculator.setTVCParameters(Vector(0,0,-0.4), Vector(0,0,1));

}