#include "KraftKontrol/modules/guidance_modules/guidance_flybywire.h"

#include "lib/MathHelperLibrary/FML.h"


void GuidanceFlyByWire::thread() {

    //if (!initialised) init();

    float dT = float(NOW() - _lastRunTimestamp)/SECONDS; //Get time delta in seconds
    _lastRunTimestamp = NOW(); //Save current run timestamp for next run.

    //Integrate speed, position and attitude
    vehicleControlSettings_.velocity += vehicleControlSettings_.linearAcceleration*dT;
    vehicleControlSettings_.position += vehicleControlSettings_.velocity*dT;

    vehicleControlSettings_.attitude = vehicleControlSettings_.attitude*FML::Quaternion<>(vehicleControlSettings_.angularRate.copy().normalize(), vehicleControlSettings_.angularRate.magnitude()*dT);

    controlSetpointTopic_.publish(vehicleControlSettings_);

}   



void GuidanceFlyByWire::init() {

    initialised = true;

    vehicleControlSettings_.velocity = 0;
    vehicleControlSettings_.acceleration = -GRAVITY_VECTOR;
    vehicleControlSettings_.linearAcceleration = 0;
    vehicleControlSettings_.angularRate = 0;

    vehicleControlSettings_.attitudeControlMode.accelerationControl = true;
    vehicleControlSettings_.attitudeControlMode.velocityControl = true;
    vehicleControlSettings_.attitudeControlMode.positionControl = true;

}
