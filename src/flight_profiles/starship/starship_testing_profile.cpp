#include "starship_testing_profile.h"



void StarshipTestingProfile::thread() {

    return; //just for vehicle testing

    if (_vehicle == nullptr) return; //Leave if no vehicle given.

    
    _vehicleSetpoint.attitude = Quaternion(1,0,0,0);
    _vehicleSetpoint.angularAcceleration = 0;
    _vehicleSetpoint.angularRate = 0;

}


void StarshipTestingProfile::init() {

    _vehicle->getControlPointer()->linkControlSetpointPointer(&_vehicleSetpoint); //Link starship control setpoint to testing setpoint.

}