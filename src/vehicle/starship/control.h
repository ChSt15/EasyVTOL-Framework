#ifndef CONTROL_H
#define CONTROL_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"


class Control {
public:

    DynamicData getDynamicsSetpoint() {return _dynamicsSetpoint;}


protected:

    void controlThread(KineticData vehicleKinetics);

    void kineticSetpoint(KineticData setpoint) {_kineticsSetpoint = setpoint;}
    

private:   

    KineticData _kineticsSetpoint;

    DynamicData _dynamicsSetpoint;

    
};





#endif