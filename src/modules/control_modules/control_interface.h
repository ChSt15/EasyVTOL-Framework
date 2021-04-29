#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"

#include "modules/guidance_modules/guidance_interface.h"


class Control_Interface {
public:

    /**
     * Returns the kinematics the system needs to achieve the desired
     * guidance inputs.
     *
     * @return dynamicSetpoint.
     */
    virtual DynamicData getDynamicsOutput() = 0;

    /**
     * Returns the a pointer to a struct with the kinematics the system 
     * needs to achieve the desired guidance inputs.
     *
     * @return dynamicSetpoint pointer.
     */
    virtual DynamicData* getDynamicsOutputPointer() = 0;

    
};





#endif