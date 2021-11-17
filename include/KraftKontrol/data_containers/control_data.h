#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H



#include "kinematic_data.h"



/**
 * This enum is for the Control setting
 */
class ControlMode {
public:

    ControlMode() {
        accelerationControl = false;
        velocityControl = false;
        positionControl = false;
    }
    
    //Control acceleration
    bool accelerationControl = false;

    //Control velocity
    bool velocityControl = false;

    //Control position
    bool positionControl = false;

};


/**
 * Inherits from Kinematic but add ability to show what needs to be controlled.
 * Used to input to control modules and output from guidance.
 */
class ControlData: public KinematicData {
public:

    //Position controller mode
    ControlMode positionControlMode;
    //Attitude controller mode
    ControlMode attitudeControlMode;
    
};



#endif