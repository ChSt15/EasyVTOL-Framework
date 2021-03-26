#ifndef STARSHIP_DYNAMICS_H
#define STARSHIP_DYNAMICS_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "definitions.h"

#include "starship_output_pins.h"

#include "outputs/rgb_led.h"

#include "utils/interval_control.h"

#include "outputs/servo_ppm.h"

#include "dynamics/servo_dynamics.h"
#include "dynamics/tvc_dynamics.h"

#include "modules/dynamics_modules/dynamics_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"


#define FLAP_MAX_VELOCITY 30
#define TOPFLAP_MAX_ACCEL 30
#define BOTTOMFLAP_MAX_ACCEL 20



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
    PPMChannel _motorCW = PPMChannel(MOTOR_PIN_CW, PPM_PROTOCOL::MULTISHOT, -1, 2);
    //CCW motor control.
    PPMChannel _motorCCW = PPMChannel(MOTOR_PIN_CCW, PPM_PROTOCOL::MULTISHOT, -1, 2);
    //Up Left flap servo.
    PPMChannel _flapUL = PPMChannel(FLAP_SERVO_PIN_UL, PPM_PROTOCOL::STANDARD_1000, 1, -1);
    ServoDynamics _flapULControl = ServoDynamics(&_flapUL, 0, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Up Right flap servo.
    PPMChannel _flapUR = PPMChannel(FLAP_SERVO_PIN_UR, PPM_PROTOCOL::STANDARD_1000, -1, 1);
    ServoDynamics _flapURControl = ServoDynamics(&_flapUR, 0, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Down Left flap servo.
    PPMChannel _flapDL = PPMChannel(FLAP_SERVO_PIN_DL, PPM_PROTOCOL::STANDARD_1000, -1, 1);
    ServoDynamics _flapDLControl = ServoDynamics(&_flapDL, 0, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);
    //Down Right flap servo.
    PPMChannel _flapDR = PPMChannel(FLAP_SERVO_PIN_DR, PPM_PROTOCOL::STANDARD_1000, 1, -1);
    ServoDynamics _flapDRControl = ServoDynamics(&_flapDR, 2*DEGREES, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);

    //Controls loop rate
    IntervalControl _interval = IntervalControl(1000);


    void _getTVCAngles(const Vector &direction, const float &twist, float &tvc1, float &tvc2, float &tvc3, float &tvc4);
    
    
};





#endif