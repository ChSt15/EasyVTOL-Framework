#include "vehicle.h"



void Vehicle::vehicleThread() {

    navigationThread();

    guidanceThread();

    kineticSetpoint(getKineticSetpoint()); //Pass kinetic parameters to control thread
    controlThread(getKineticData());

    setDynamics(getDynamicsSetpoint()); //Pass dynamic parameters to dynamics control thread
    dynamicsThread(getKineticData());

}