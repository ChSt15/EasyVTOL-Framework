#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"

#include "flight_settings.h"


class ControlTemplate {
public:

    /**
     * Returns the dynamic setpoints the vehicle has to follow 
     * to achieve the commands.
     *
     * @param values none.
     * @return dynamic setpoint in local coordinate system.
     */
    virtual DynamicData getControlDynamicsSetpoint();


protected:

    virtual void controlThread(KineticData vehicleKinetics);
    virtual void controlInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);
    
    /**
     * Sets the kinetics of which the vehicle should have.
     *
     * @param values setpoint in world coordinate system.
     * @return none.
     */
    virtual void setControlKineticSetpoint(KineticData setpoint);

    
};





#endif