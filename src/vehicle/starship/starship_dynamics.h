#ifndef STARSHIP_DYNAMICS_H
#define STARSHIP_DYNAMICS_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "definitions.h"

#include "starship_properties.h"
#include "starship_data.h"

#include "outputs/rgb_led.h"

#include "utils/interval_control.h"

#include "outputs/servo_ppm.h"

#include "dynamics/servo_dynamics.h"
#include "dynamics/tvc_dynamics.h"

#include "modules/dynamics_modules/dynamics_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"


//Flap servo accel and velocity maximums
#define FLAP_MAX_VELOCITY 5
#define TOPFLAP_MAX_ACCEL 30
#define BOTTOMFLAP_MAX_ACCEL 30

//TVC constraints
#define MAX_TVC_ANGLE 45*DEGREES
#define MAX_TVC_FORCE 20 //In newtons

//Actuator mapping for manual control
#define STARSHIP_ACTUATOR_FLAPUL 0
#define STARSHIP_ACTUATOR_FLAPUR 1
#define STARSHIP_ACTUATOR_FLAPDR 2
#define STARSHIP_ACTUATOR_FLAPDL 3
#define STARSHIP_ACTUATOR_TVC1 4
#define STARSHIP_ACTUATOR_TVC2 5
#define STARSHIP_ACTUATOR_TVC3 6
#define STARSHIP_ACTUATOR_TVC4 7
#define STARSHIP_ACTUATOR_MOTORCW 8
#define STARSHIP_ACTUATOR_MOTORCCW 9



class StarshipDynamics: public Dynamics {
public:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Tells starship which actuators to use 
     *
     * @param values STARSHIP_MODE.
     * @return none.
     */
    void setActuatorMode(STARSHIP_MODE starshipMode) {
        _starshipMode = starshipMode;
    }
    

private:

    //TVC servo in X+
    PPMChannel _TVCServo1 = PPMChannel(TVC_SERVO_PIN_1, PPM_PROTOCOL::STANDARD_1000, 0, -1);
    //TVC servo in Y+
    PPMChannel _TVCServo2 = PPMChannel(TVC_SERVO_PIN_2, PPM_PROTOCOL::STANDARD_1000, 0, -1);
    //TVC servo in X-
    PPMChannel _TVCServo3 = PPMChannel(TVC_SERVO_PIN_3, PPM_PROTOCOL::STANDARD_1000, 0, 1);
    //TVC servo in Y-
    PPMChannel _TVCServo4 = PPMChannel(TVC_SERVO_PIN_4, PPM_PROTOCOL::STANDARD_1000, -0.1, 1);
    //CW motor control.
    PPMChannel _motorCW = PPMChannel(MOTOR_PIN_CW, PPM_PROTOCOL::ONESHOT_125, -1, 2);
    //CCW motor control.
    PPMChannel _motorCCW = PPMChannel(MOTOR_PIN_CCW, PPM_PROTOCOL::ONESHOT_125, -1, 2);
    //Up Left flap servo.
    PPMChannel _flapUL = PPMChannel(FLAP_SERVO_PIN_UL, PPM_PROTOCOL::STANDARD_1000, 1, -1);
    ServoDynamics _flapULControl = ServoDynamics(&_flapUL, 1*DEGREES, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Up Right flap servo.
    PPMChannel _flapUR = PPMChannel(FLAP_SERVO_PIN_UR, PPM_PROTOCOL::STANDARD_1000, -1, 1);
    ServoDynamics _flapURControl = ServoDynamics(&_flapUR, -5*DEGREES, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Down Left flap servo.
    PPMChannel _flapDL = PPMChannel(FLAP_SERVO_PIN_DL, PPM_PROTOCOL::STANDARD_1000, -1, 1);
    ServoDynamics _flapDLControl = ServoDynamics(&_flapDL, -7*DEGREES, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);
    //Down Right flap servo.
    PPMChannel _flapDR = PPMChannel(FLAP_SERVO_PIN_DR, PPM_PROTOCOL::STANDARD_1000, 1, -1);
    ServoDynamics _flapDRControl = ServoDynamics(&_flapDR, 1*DEGREES, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);

    //Controls loop rate
    IntervalControl _interval = IntervalControl(1000);

    //Enum for flap testing
    enum FLAP_TEST_STAGE {
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT
    };

    FLAP_TEST_STAGE _flapTestStage = FLAP_TEST_STAGE::TOP_LEFT;
    uint8_t _flapTestCounter = 0;

    //Tells system which actuators to use
    STARSHIP_MODE _starshipMode = STARSHIP_MODE::TVC_CONTROL;

    //Helps with calculating TVC stuff
    TVCDynamics _TVCCalculator = TVCDynamics(Vector(0,0,-0.4), Vector(0,0,-1));


    void _getTVCAngles(const Vector &direction, const float &twist, float &tvc1, float &tvc2, float &tvc3, float &tvc4);
    
    
};





#endif