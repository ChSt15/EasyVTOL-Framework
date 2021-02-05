#include "vehicle.h"



void Vehicle::vehicleThread() {

    if (!interval.isTimeToRun()) return; //Leave if its not time to run yet

    navigationThread();

    guidanceThread();

    kineticSetpoint(getKineticSetpoint()); //Pass kinetic parameters to control thread
    controlThread(getKineticData());

    setDynamics(getDynamicsSetpoint()); //Pass dynamic parameters to dynamics control thread
    dynamicsThread(getKineticData());

}