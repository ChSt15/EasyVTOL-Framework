#include "vehicle.h"



void Vehicle::vehicleThread() {

    if (!interval.isTimeToRun()) return; //Leave if its not time to run yet

    if (!_vehicleInitialized) vehicleInit();

    navigationThread();
    KineticData kineticData = getNavigationKineticData(); //retrieve vehicle kinematics

    guidanceThread();

    setControlKineticSetpoint(getGuidanceKineticSetpoint()); //Pass kinetic parameters to control thread
    controlThread(kineticData);

    setDynamicsSetpoint(getControlDynamicsSetpoint()); //Pass dynamic parameters to dynamics control thread
    dynamicsThread(kineticData);

    setActuatorSetpoints(getActuatorSetpoints()); //Pass actuator setpoints to output control thread
    controlThread(kineticData);

}


void Vehicle::vehicleInit() {
    guidanceInit(&_flightMode, &_flightProfile);
    navigationInit(&_flightMode, &_flightProfile);
    controlInit(&_flightMode, &_flightProfile);
    dynamicsInit(&_flightMode, &_flightProfile);
    outputControlInit(&_flightMode, &_flightProfile);
}