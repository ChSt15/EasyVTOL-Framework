#ifndef STARSHIP_DYNAMICS_H
#define STARSHIP_DYNAMICS_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "definitions.h"

#include "starship_output_pins.h"

#include "utils/interval_control.h"

#include "outputs/servo_ppm.h"

#include "dynamics/servo_dynamics.h"
#include "dynamics/tvc_dynamics.h"

#include "modules/dynamics_modules/dynamics_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"


#define FLAP_MAX_VELOCITY 4
#define TOPFLAP_MAX_ACCEL 2
#define BOTTOMFLAP_MAX_ACCEL 2



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

    //TVC servos. 0 is x+, 1 is y+, 2 is x-, 3 is y-.
    PPMChannel TVCServo1 = PPMChannel(TVC_SERVO_PIN_1, PPM_PROTOCOL::STANDARD_1000);
    PPMChannel TVCServo2 = PPMChannel(TVC_SERVO_PIN_2, PPM_PROTOCOL::STANDARD_1000);
    PPMChannel TVCServo3 = PPMChannel(TVC_SERVO_PIN_3, PPM_PROTOCOL::STANDARD_1000);
    PPMChannel TVCServo4 = PPMChannel(TVC_SERVO_PIN_4, PPM_PROTOCOL::STANDARD_1000);
    //CW motor control.
    PPMChannel motorCW = PPMChannel(MOTOR_PIN_CW, PPM_PROTOCOL::ONESHOT_125);
    //CCW motor control.
    PPMChannel motorCCW = PPMChannel(MOTOR_PIN_CCW, PPM_PROTOCOL::ONESHOT_125);
    //Up Left flap servo.
    PPMChannel flapUL = PPMChannel(FLAP_SERVO_PIN_UL, PPM_PROTOCOL::STANDARD_1000);
    ServoDynamics flapULControl = ServoDynamics(&flapUL, 1/(90*DEGREES), 0, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Up Right flap servo.
    PPMChannel flapUR = PPMChannel(FLAP_SERVO_PIN_UR, PPM_PROTOCOL::STANDARD_1000);
    ServoDynamics flapURControl = ServoDynamics(&flapUR, 1/(90*DEGREES), 0, FLAP_MAX_VELOCITY, TOPFLAP_MAX_ACCEL);
    //Down Left flap servo.
    PPMChannel flapDL = PPMChannel(FLAP_SERVO_PIN_DL, PPM_PROTOCOL::STANDARD_1000);
    ServoDynamics flapDLControl = ServoDynamics(&flapDL, 1/(90*DEGREES), 0, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);
    //Down Right flap servo.
    PPMChannel flapDR = PPMChannel(FLAP_SERVO_PIN_DR, PPM_PROTOCOL::STANDARD_1000);
    ServoDynamics flapDRControl = ServoDynamics(&flapDR, 1/(90*DEGREES), 0, FLAP_MAX_VELOCITY, BOTTOMFLAP_MAX_ACCEL);

    IntervalControl _interval = IntervalControl(1000);
    
    
};





#endif