#ifndef CONTROL_H
#define CONTROL_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"

#include "flight_modes.h"


class Control {
public:

    /**
     * Returns the dynamic setpoints the vehicle has to follow 
     * to achieve the commands.
     *
     * @param values none.
     * @return dynamic setpoint in local coordinate system.
     */
    DynamicData getControlDynamicsSetpoint() {return _dynamicsSetpoint;}


protected:

    void controlThread(KineticData vehicleKinetics);
    void controlInit(FLIGHT_MODE* flightModePointer);
    
    /**
     * Sets the kinetics of which the vehicle should have.
     *
     * @param values setpoint in world coordinate system.
     * @return none.
     */
    void setControlKineticSetpoint(KineticData setpoint) {_kineticsSetpoint = setpoint;}
    

private:   

    KineticData _kineticsSetpoint;

    DynamicData _dynamicsSetpoint;

    FLIGHT_MODE* _flightMode;

    
};





#endif