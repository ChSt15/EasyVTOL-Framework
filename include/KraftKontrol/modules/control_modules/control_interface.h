#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "KraftKontrol/data_containers/control_data.h"
#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/guidance_modules/guidance_interface.h"
#include "KraftKontrol/modules/navigation_modules/navigation_interface.h"


class Control_Interface {
public:

    /**
     * Sets the control modules guidance module.
     * @param guidanceModule Pointer to module to use.
     */
    virtual void setGuidanceModule(Guidance_Interface* guidanceModule) = 0;

    /**
     * Sets the control modules navigation module.
     * @param navigationModule Pointer to module to use.
     */
    virtual void setGuidanceModule(Navigation_Interface* navigationModule) = 0;

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

    /**
     * Resets control module. Things like PID I values are set to 0.
     */
    virtual void reset() = 0;

    
};





#endif