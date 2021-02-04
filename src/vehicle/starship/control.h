#ifndef CONTROL_H
#define CONTROL_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"


class Control {
public:

    void kineticSetpoint(KineticData setpoint) {_kineticsSetpoint = setpoint;}

    DynamicData getDynamicsSetpoint() {return _dynamicsSetpoint;}


protected:

    void controlThread(KineticData vehicleKinetics);
    

private:   

    KineticData _kineticsSetpoint;

    DynamicData _dynamicsSetpoint;

    
};





#endif