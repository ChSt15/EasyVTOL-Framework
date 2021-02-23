#include "starship.h"



void Starship::thread() {

    if (!_interval.isTimeToRun()) return; //Leave if its not time to run yet

    if (!_vehicleInitialized) init();

    //Serial.println("Test: " + String(_navigation->getPositionAccuracy()));

    //_navigation->thread(*this);
    _vehicleKinematics = _navigation->getKinematicData(); //retrieve vehicle kinematics

    /*guidanceThread();

    setControlKineticSetpoint(getGuidanceKineticSetpoint()); //Pass kinetic parameters to control thread
    controlThread(kineticData);

    setDynamicsSetpoint(getControlDynamicsSetpoint()); //Pass dynamic parameters to dynamics control thread
    dynamicsThread(kineticData);

    setActuatorSetpoints(getActuatorSetpoints()); //Pass actuator setpoints to output control thread
    controlThread(kineticData);*/

}


void Starship::init() {
    _navigation->init(); //init the navigation module and pass pointer to vehicle data.
}