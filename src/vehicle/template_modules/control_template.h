#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "Arduino.h"

#include "vehicle/dynamic_data.h"
#include "vehicle/kinetic_data.h"
#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"


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

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void controlThread(KineticData vehicleKinetics);

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
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