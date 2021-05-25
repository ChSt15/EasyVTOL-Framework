#include "guidance_flybywire.h"



void GuidanceFlyByWire::thread() {

    if (!initialised) init();

    float dT = ((float) micros() - _lastRunTimestamp)/1000000.0f; //Get time delta in seconds
    _lastRunTimestamp = micros(); //Save current run timestamp for next run.

    //Integrate speed, position and attitude
    vehicleControlSettings_.velocity += vehicleControlSettings_.linearAcceleration*dT;
    vehicleControlSettings_.position += vehicleControlSettings_.velocity*dT;

    vehicleControlSettings_.attitude = vehicleControlSettings_.attitude*Quaternion<>(vehicleControlSettings_.angularRate.copy().normalize(), vehicleControlSettings_.angularRate.magnitude()*dT);

}   



void GuidanceFlyByWire::init() {

    initialised = true;

    vehicleControlSettings_.velocity = 0;
    vehicleControlSettings_.acceleration = -GRAVITY_VECTOR;
    vehicleControlSettings_.linearAcceleration = 0;
    vehicleControlSettings_.angularRate = 0;

    vehicleControlSettings_.attitudeControlMode = eControlMode_t::eControlMode_Acceleration_Velocity_Position;

}
