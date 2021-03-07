#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"


class Control: public Module {
public:

    /**
     * Sets the control setpoint from a guidance module.
     * 
     * Use linkControlSetpointPointer() for data linking.
     * 
     * Calling this will remove the link. Data must be linked again!
     *
     * @param values control parameters.
     * @return none.
     */
    void setControlSetpoint(const ControlData &controlSetpoint) {
        _controlSetpoint = &_controlSetpointDefault; //Make sure pointer is set to internal data so to not change linked data.
        *_controlSetpoint = controlSetpoint;
    };

    /**
     * Sets the control input pointer to the inputed pointer 
     * from a guidance module.
     * 
     * Allows the control module to automatically retrieve its needed
     * data from the pointer.
     * 
     * This must only be called once.
     * 
     * returns false if failed from null pointer input.
     *
     * @param values control parameters.
     * @return bool.
     */
    bool linkControlSetpointPointer(ControlData *controlDataPointer) {
        if (controlDataPointer == nullptr) return false;
        _controlSetpoint = controlDataPointer;
        return true;
    };

    /**
     * Sets the navigation input data from a navigation module.
     * 
     * Use linkNavigationDataPointer() for data linking.
     * 
     * Calling this will remove the link. Data must be linked again!
     *
     * @param values navigation data parameters.
     * @return none.
     */
    void setNavigationInput(const NavigationData &navigationData) {
        _navigationData = &_navigationDataDefault; //Make sure pointer is set to internal data so to not change linked data.
        *_navigationData = navigationData;
    };

    /**
     * Sets the navigation data pointer to the inputed pointer 
     * from a navigation module.
     * 
     * Allows the dynamics module to automatically retrieve its needed
     * data from the pointer.
     * 
     * This must only be called once.
     * 
     * returns false if null pointer was given.
     *
     * @param values navigation data pointer.
     * @return bool.
     */
    bool linkNavigationDataPointer(NavigationData *navigationDataPointer) {
        if (navigationDataPointer == nullptr) return false;
        _navigationData = navigationDataPointer;
        return true;
    };

    /**
     * Returns the kinematics the system needs to achieve the desired
     * guidance inputs.
     *
     * @param values none.
     * @return kinematicSetpoint.
     */
    virtual DynamicData getDynamicsOutput() {return _controlOutput;};

    /**
     * Returns the a pointer to a struct with the kinematics the system 
     * needs to achieve the desired guidance inputs.
     *
     * @param values none.
     * @return kinematicSetpoint pointer.
     */
    virtual DynamicData* getDynamicsOutputPointer() {return &_controlOutput;};


protected:

    static ControlData* _controlSetpoint;
    static NavigationData* _navigationData;

    static DynamicData _controlOutput;


private:

    static ControlData _controlSetpointDefault;
    static NavigationData _navigationDataDefault;

    
};





#endif