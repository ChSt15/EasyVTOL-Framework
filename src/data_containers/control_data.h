#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H


#include "definitions.h"

#include "3d_math.h"

#include "kinematic_data.h"



/**
 * This enum is for the Control setting
 */
enum CONTROL_MODE {
    //Disable control.
    CONTROL_DISABLED,
    //Control acceleration/rate acceleration
    CONTROL_ACCELERATION,
    //Control velocity/rate
    CONTROL_VELOCITY,
    //Control both velocity/rate and acceleration/rate acceleration
    CONTROL_ACCELERATION_VELOCITY,
    //Control position
    CONTROL_POSITION,
    //Control velocity/rate and position
    CONTROL_VELOCITY_POSITION,
    //Control acceleration/rate acceleration and velocity/rate and position
    CONTROL_ACCELERATION_VELOCITY_POSITION
};


/**
 * Inherits from Kinematic but add ability to show what needs to be controlled.
 * Used to input to control modules and output from guidance.
 */
struct ControlData: public KinematicData {

    CONTROL_MODE positionControlMode = CONTROL_MODE::CONTROL_DISABLED;
    CONTROL_MODE attitudeControlMode = CONTROL_MODE::CONTROL_DISABLED;
    
};



#endif