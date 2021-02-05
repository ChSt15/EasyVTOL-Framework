#ifndef CONTROL_H
#define CONTROL_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"


class Control {
public:

    /**
     * Returns the dynamic setpoints the vehicle has to follow
     * to achieve the commands.
     *
     * @param values none.
     * @return dynamic setpoint in local coordinate system.
     */
    DynamicData getDynamicsSetpoint() {return _dynamicsSetpoint;}


protected:

    void controlThread(KineticData vehicleKinetics);
    
    /**
     * Sets the kinetics of which the vehicle should have.
     *
     * @param values setpoint in world coordinate system.
     * @return none.
     */
    void kineticSetpoint(KineticData setpoint) {_kineticsSetpoint = setpoint;}
    

private:   

    KineticData _kineticsSetpoint;

    DynamicData _dynamicsSetpoint;

    
};





#endif