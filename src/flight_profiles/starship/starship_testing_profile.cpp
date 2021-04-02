#include "starship_testing_profile.h"



void StarshipTestingProfile::thread() {

    if (_vehicle == nullptr) return; //Leave if no vehicle given.

    
    _vehicleSetpoint.attitude = Quaternion(1,0,0,0);
    _vehicleSetpoint.angularAcceleration = 0;
    _vehicleSetpoint.angularRate = 0;

    _vehicleSetpoint.attitudeControlMode = CONTROL_MODE::CONTROL_ACCELERATION_VELOCITY_POSITION;

    _vehicle->getControlPointer()->setControlSetpoint(_vehicleSetpoint);

}


void StarshipTestingProfile::init() {

    //_vehicle->getControlPointer()->linkControlSetpointPointer(&_vehicleSetpoint); //Link starship control setpoint to testing setpoint.

}