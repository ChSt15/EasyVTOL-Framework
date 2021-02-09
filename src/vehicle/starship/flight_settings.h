#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

/**
 * Enum containing all flight modes
 */
enum FLIGHT_MODE {
    //Disarms vehicle disabling motors and moving acuators into a home postion
    DISARMED,
    //Arms vehicle by enabling motors(at lowest throttle) and putting all actuators into flight ready postion
    ARMED,
    //Immediatly shuts everything down. Motors are disabled and actators stop moving.
    FAILSAFE,
    //Makes vehicle only follow rate commands.
    RATE_CONTROL,
    //Makes vehicle only follow attitude and rate commands.
    ATTITUDE_CONTROL,
    //Makes vehicle only follow velocity commands.
    VELOCITY_CONTROL,
    //Makes vehicle only follow position commands.
    POSITION_CONTROL
};


/**
 * Enum containing all flight profiles
 */
enum FLIGHT_PROFILE {
    //Goes into "normal" rocket flight but can also hover at given point
    HOVER,
    //Goes into belly flop. Meaning side of vehicle windwards.
    BELLY_FLOP,
};



#endif
