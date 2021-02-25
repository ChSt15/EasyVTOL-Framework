#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H


#include "definitions.h"

#include "vector_math.h"
#include "quaternion_math.h"

#include "kinematic_data.h"



/**
 * This enum is for the Control setting
 */
enum CONTROL_MODE {
    //Disable control.
    CONTROL_DISABLED,
    //Control velocity/rate
    CONTROL_VELOCITY,
    //Control position
    CONTROL_POSITION,
    //Control velocity/rate and position
    CONTROL_VELOCITY_POSITION
};


/**
 * Inherits from Kinematic but add ability to show what needs to be controlled.
 * Used to input to control modules and output from guidance.
 */
struct ControlData: public KinematicData {

    CONTROL_MODE controlMode = CONTROL_MODE::CONTROL_DISABLED;
    
};



#endif