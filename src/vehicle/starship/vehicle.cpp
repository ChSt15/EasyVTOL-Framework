#include "vehicle.h"



void Vehicle::vehicleThread() {

    if (!interval.isTimeToRun()) return; //Leave if its not time to run yet

    if (!_vehicleInitialized) vehicleInit();

    navigationThread();

    guidanceThread();

    setControlKineticSetpoint(getGuidanceKineticSetpoint()); //Pass kinetic parameters to control thread
    controlThread(getNavigationKineticData());

    setDynamicsSetpoint(getControlDynamicsSetpoint()); //Pass dynamic parameters to dynamics control thread
    dynamicsThread(getNavigationKineticData());

}


void Vehicle::vehicleInit() {
    controlInit(&_flightMode, &_flightProfile);
    dynamicsInit(&_flightMode, &_flightProfile);
    navigationInit(&_flightMode, &_flightProfile);
    guidanceInit(&_flightMode, &_flightProfile);
}