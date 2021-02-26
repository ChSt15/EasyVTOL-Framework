#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "data_containers/control_data.h"



class Guidance: public Module {
public:

    /**
     * Returns a struct containing all the vehicles
     * setpoint data that feeds to the controller.
     *
     * @param values none.
     * @return control parameters.
     */
    virtual ControlData getControlSetpoint() {return _vehicleControlSettings;};

    /**
     * Returns a pointer to a struct containing all the 
     * vehicles setpoint data that feeds to the controller.
     * 
     * Using pointers allows for data linking instead of
     * always passing data.
     *
     * @param values none.
     * @return control parameter pointer.
     */
    virtual ControlData* getControlSetpointPointer() {return &_vehicleControlSettings;};

    /**
     * Returns distance from endpoint. Used for switching between guidance modules.
     *
     * @param values none.
     * @return float.
     */
    virtual float distanceFromEndpoint() = 0;


protected:

    /**
     * This pointer points to the global navigation data kinematic data container
     */
    static ControlData _vehicleControlSettings;

};





#endif