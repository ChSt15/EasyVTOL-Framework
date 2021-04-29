#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "data_containers/control_data.h"



class Guidance_Interface {
public:

    /**
     * Returns a struct containing all the vehicles
     * setpoint data that feeds to the controller.
     *
     * @return control parameters.
     */
    virtual ControlData getControlSetpoint() = 0;

    /**
     * Returns a pointer to a struct containing all the 
     * vehicles setpoint data that feeds to the controller.
     * 
     * Using pointers allows for data linking instead of
     * always passing data.
     *
     * @return control parameter pointer.
     */
    virtual ControlData* getControlSetpointPointer() = 0;


protected:

    

};





#endif