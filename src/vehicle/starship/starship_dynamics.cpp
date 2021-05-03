#include "starship_dynamics.h"



void StarshipDynamics::thread() {


    if (!initialised_) init();


    static uint32_t lastswitch = 0;
    static float angle = 0;


    if (actuatorStatusSetpoint_ == eActuatorStatus_t::eActuatorStatus_Enabled) {

        if (actuatorStatusLast_ != actuatorStatusSetpoint_) { //Will be ran once at state change
            actuatorStatusLast_ = actuatorStatusSetpoint_;

            flapULControl_.setParameters(FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
            flapURControl_.setParameters(FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
            flapDRControl_.setParameters(FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
            flapDLControl_.setParameters(FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);

            TVCServo1_.activateChannel();
            TVCServo2_.activateChannel();
            TVCServo3_.activateChannel();
            TVCServo4_.activateChannel();

            actuatorStatus_ = eActuatorStatus_t::eActuatorStatus_Enabled;

        }

        if (!actuatorManualMode_ && !actuatorTesting_) {   

            DynamicData dynamicSetpoint = controlModule_->getDynamicsOutput();

            float force = 0;
            Vector directionBuf = Vector(0,0,1);

            TVCCalculator_.dynamicsSetpoint(dynamicSetpoint);
            TVCCalculator_.getTVCSettings(force, directionBuf);

            Vector direction;
            direction.z = directionBuf.z;
            direction.x = -directionBuf.y;
            direction.y = directionBuf.x;

            //calculate TVC angles
            float TVC1, TVC2, TVC3, TVC4;
            float twist = -constrain(dynamicSetpoint.torqe.z, -45*DEGREES, 45*DEGREES);
            getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);


            TVCServo1_.setAngle(TVC1);
            TVCServo2_.setAngle(TVC2);
            TVCServo3_.setAngle(TVC3);
            TVCServo4_.setAngle(TVC4);

            flapULControl_.setPosition(0*DEGREES);
            flapURControl_.setPosition(0*DEGREES);
            flapDRControl_.setPosition(0*DEGREES);
            flapDLControl_.setPosition(0*DEGREES);

            motorCW_.setChannel(force/MAX_TVC_FORCE);
            motorCCW_.setChannel(force/MAX_TVC_FORCE);

        } else if (actuatorManualMode_ && !actuatorTesting_) {

            flapULControl_.setPosition(actuatorManualSetpoint_.actuatorSetting[0]);
            flapURControl_.setPosition(actuatorManualSetpoint_.actuatorSetting[1]);
            flapDRControl_.setPosition(actuatorManualSetpoint_.actuatorSetting[2]);
            flapDLControl_.setPosition(actuatorManualSetpoint_.actuatorSetting[3]);
            TVCServo1_.setChannel(actuatorManualSetpoint_.actuatorSetting[4]);
            TVCServo2_.setChannel(actuatorManualSetpoint_.actuatorSetting[5]);
            TVCServo3_.setChannel(actuatorManualSetpoint_.actuatorSetting[6]);
            TVCServo4_.setChannel(actuatorManualSetpoint_.actuatorSetting[7]);
            motorCW_.setChannel(actuatorManualSetpoint_.actuatorSetting[8]);
            motorCCW_.setChannel(actuatorManualSetpoint_.actuatorSetting[9]);

        } else if (actuatorTesting_ && !actuatorManualMode_) { //Actuator testing mode

            if (millis() - lastswitch >= 1000) {
                lastswitch = millis();

                if (angle == 90.0f*DEGREES) {
                    angle = 0.0f*DEGREES;
                } else {
                    angle = 90.0f*DEGREES;
                    flapTestCounter_++;
                }

            }

            switch (flapTestStage_) {

            case FLAP_TEST_STAGE::TOP_LEFT:
                flapULControl_.setPosition(angle);
                flapURControl_.setPosition(0);
                flapDRControl_.setPosition(0);
                flapDLControl_.setPosition(0);
                if (flapTestCounter_ > 1) {
                    flapTestCounter_ = 0;
                    flapTestStage_ = FLAP_TEST_STAGE::TOP_RIGHT;
                }
                break;

            case FLAP_TEST_STAGE::TOP_RIGHT:
                flapULControl_.setPosition(0);
                flapURControl_.setPosition(angle);
                flapDRControl_.setPosition(0);
                flapDLControl_.setPosition(0);
                if (flapTestCounter_ > 2) {
                    flapTestCounter_ = 0;
                    flapTestStage_ = FLAP_TEST_STAGE::BOTTOM_LEFT;
                }
                break;

            case FLAP_TEST_STAGE::BOTTOM_LEFT:
                flapULControl_.setPosition(0);
                flapURControl_.setPosition(0);
                flapDRControl_.setPosition(0);
                flapDLControl_.setPosition(angle);
                if (flapTestCounter_ > 3) {
                    flapTestCounter_ = 0;
                    flapTestStage_ = FLAP_TEST_STAGE::BOTTOM_RIGHT;
                }
                break;

            case FLAP_TEST_STAGE::BOTTOM_RIGHT:
                flapULControl_.setPosition(0);
                flapURControl_.setPosition(0);
                flapDRControl_.setPosition(angle);
                flapDLControl_.setPosition(0);
                if (flapTestCounter_ > 4) {
                    flapTestCounter_ = 0;
                    flapTestStage_ = FLAP_TEST_STAGE::TOP_LEFT;
                }
                break;
            
            default:
                break;
            }

            Vector direction = Quaternion(Vector(0,0,1), float(millis())/1000.0f).rotateVector(Vector(1,0,1));//_navigationData->attitude.copy().conjugate().rotateVector(Vector(0,0,1));

            //calculate TVC angles
            float TVC1, TVC2, TVC3, TVC4;
            float twist = 45*DEGREES*sin((float)millis()/300);
            getTVCAngles(direction, twist, TVC1, TVC2, TVC3, TVC4);

            TVCServo1_.setAngle(TVC1);
            TVCServo2_.setAngle(TVC2);
            TVCServo3_.setAngle(TVC3);
            TVCServo4_.setAngle(TVC4);

            motorCW_.setChannel(0);
            motorCCW_.setChannel(0);
            
        }

    } else if (actuatorStatusSetpoint_ == eActuatorStatus_t::eActuatorStatus_Ready) {

        if (actuatorStatusLast_ != actuatorStatusSetpoint_) { //Will be ran once at state change
            actuatorStatusLast_ = actuatorStatusSetpoint_;

            flapULControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapURControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapDRControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapDLControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);

            flapULControl_.setPosition(0);
            flapURControl_.setPosition(0);
            flapDRControl_.setPosition(0);
            flapDLControl_.setPosition(0);

            TVCServo1_.activateChannel();
            TVCServo2_.activateChannel();
            TVCServo3_.activateChannel();
            TVCServo4_.activateChannel();

        }

        TVCServo1_.setAngle(0);
        TVCServo2_.setAngle(0);
        TVCServo3_.setAngle(0);
        TVCServo4_.setAngle(0);

        motorCW_.setChannel(0);
        motorCCW_.setChannel(0);

        //Get sum of positions. Should be at 0 when all flaps are in position
        float position = flapULControl_.getPosition() + flapURControl_.getPosition() + flapDRControl_.getPosition() + flapDLControl_.getPosition();

        if (position < 0.1*DEGREES) {
            actuatorStatus_ = eActuatorStatus_t::eActuatorStatus_Ready;
        }

    } else if (actuatorStatusSetpoint_ == eActuatorStatus_t::eActuatorStatus_Disabled) {

        if (actuatorStatusLast_ != actuatorStatusSetpoint_) { //Will be ran once at state change
            actuatorStatusLast_ = actuatorStatusSetpoint_;

            flapULControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapURControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapDRControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);
            flapDLControl_.setParameters(FLAP_START_MAX_VELOCITY, FLAP_START_MAX_ACCEL);

            flapULControl_.setPosition(90*DEGREES);
            flapURControl_.setPosition(90*DEGREES);
            flapDRControl_.setPosition(90*DEGREES);
            flapDLControl_.setPosition(90*DEGREES);

            TVCServo1_.activateChannel(false);
            TVCServo2_.activateChannel(false);
            TVCServo3_.activateChannel(false);
            TVCServo4_.activateChannel(false);

            actuatorStatus_ = eActuatorStatus_t::eActuatorStatus_Disabled;

        }

        motorCW_.setChannel(0);
        motorCCW_.setChannel(0);

    }

    flapDLControl_.thread();
    flapULControl_.thread();
    flapDRControl_.thread();
    flapURControl_.thread();

}



void StarshipDynamics::getTVCAngles(const Vector &direction, const float &twist, float &tvc1, float &tvc2, float &tvc3, float &tvc4) {

    tvc1 = atan2(direction.x, direction.z);
    tvc2 = atan2(direction.y, direction.z);
    tvc3 = tvc1 - twist;
    tvc4 = tvc2 - twist;

    tvc1 += twist;
    tvc2 += twist;

}



void StarshipDynamics::init() {

    initialised_ = true;

    flapDLControl_.setPosition(90*DEGREES);
    flapULControl_.setPosition(90*DEGREES);
    flapDRControl_.setPosition(90*DEGREES);
    flapURControl_.setPosition(90*DEGREES);

    TVCServo1_.activateChannel();
    TVCServo2_.activateChannel();
    TVCServo3_.activateChannel();
    TVCServo4_.activateChannel();
    flapDL_.activateChannel();
    flapUL_.activateChannel();
    flapDR_.activateChannel();
    flapUR_.activateChannel();

    motorCW_.activateChannel();
    motorCCW_.activateChannel();

    motorCW_.setChannel(0);
    motorCCW_.setChannel(0);

    TVCCalculator_.setDynamicConstraints(MAX_TVC_FORCE, MAX_TVC_ANGLE);
    TVCCalculator_.setTVCParameters(Vector(0,0,-0.4), Vector(0,0,1));

}