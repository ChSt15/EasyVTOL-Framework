#ifndef eControlMode_DATA_H
#define eControlMode_DATA_H



#include "kinematic_data.h"



/**
 * This enum is for the Control setting
 */
enum eControlMode_t {
    //Disable control.
    eControlMode_Disable,
    //Control acceleration/rate acceleration
    eControlMode_Acceleration,
    //Control velocity/rate
    eControlMode_Velocity,
    //Control both velocity/rate and acceleration/rate acceleration
    eControlMode_Acceleration_Velocity,
    //Control position
    eControlMode_Position,
    //Control velocity/rate and position
    eControlMode_Velocity_Position,
    //Control acceleration/rate acceleration and velocity/rate and position
    eControlMode_Acceleration_Velocity_Position
};


/**
 * Inherits from Kinematic but add ability to show what needs to be controlled.
 * Used to input to control modules and output from guidance.
 */
struct ControlData: public KinematicData {

    //Position controller mode
    eControlMode_t positionControlMode = eControlMode_t::eControlMode_Disable;
    //Attitude controller mode
    eControlMode_t attitudeControlMode = eControlMode_t::eControlMode_Disable;
    
};



#endif