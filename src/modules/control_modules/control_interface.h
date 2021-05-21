#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"

#include "modules/guidance_modules/guidance_interface.h"
#include "modules/navigation_modules/navigation_interface.h"


class Control_Interface {
public:

    /**
     * Sets the control modules guidance module.
     * @param guidanceModule Pointer to module to use.
     */
    inline virtual void setGuidanceModule(Guidance_Interface* guidanceModule) = 0;

    /**
     * Sets the control modules navigation module.
     * @param navigationModule Pointer to module to use.
     */
    inline virtual void setGuidanceModule(Navigation_Interface* navigationModule) = 0;

    /**
     * Returns the kinematics the system needs to achieve the desired
     * guidance inputs.
     *
     * @return dynamicSetpoint.
     */
    inline virtual DynamicData getDynamicsOutput() = 0;

    /**
     * Returns the a pointer to a struct with the kinematics the system 
     * needs to achieve the desired guidance inputs.
     *
     * @return dynamicSetpoint pointer.
     */
    inline virtual DynamicData* getDynamicsOutputPointer() = 0;

    
};





#endif