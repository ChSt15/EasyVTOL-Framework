#ifndef STARSHIP_PROPERTIES_H
#define STARSHIP_PROPERTIES_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#define TVC_SERVO_PIN_1 1
#define TVC_SERVO_PIN_2 22
#define TVC_SERVO_PIN_3 0
#define TVC_SERVO_PIN_4 23

#define FLAP_SERVO_PIN_UL 25
#define FLAP_SERVO_PIN_UR 24
#define FLAP_SERVO_PIN_DL 29
#define FLAP_SERVO_PIN_DR 28

#define MOTOR_PIN_CW 8
#define MOTOR_PIN_CCW 7



#endif