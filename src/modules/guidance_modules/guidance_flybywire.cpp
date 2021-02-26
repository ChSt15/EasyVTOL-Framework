#include "guidance_flybywire.h"



void GuidanceFlyByWire::thread() {

    uint32_t dTimeus;

    if(!_interval.isTimeToRun(dTimeus)) return; //Check if its time to run

    float dT = dTimeus/1000000.0f; //Get time delta in seconds

    //Integrate speed, position and attitude
    _vehicleControlSettings.velocity += _vehicleControlSettings.linearAcceleration*dT;
    _vehicleControlSettings.position += _vehicleControlSettings.velocity*dT;

    _vehicleControlSettings.attitude = _vehicleControlSettings.attitude*Quaternion(_vehicleControlSettings.angularRate.copy().normalize(), _vehicleControlSettings.angularRate.magnitude()*dT);

}



void GuidanceFlyByWire::init() {

    _vehicleControlSettings.velocity = 0;
    _vehicleControlSettings.acceleration = GRAVITY_VECTOR;
    _vehicleControlSettings.linearAcceleration = 0;
    _vehicleControlSettings.angularRate = 0;

}
