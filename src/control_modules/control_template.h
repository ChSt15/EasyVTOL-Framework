#ifndef CONTROL_TEMPLATE_H
#define CONTROL_TEMPLATE_H



#include "templates/module_template.h"

#include "data_containers/control_data.h"
#include "data_containers/navigation_data.h"


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
     * @param values control parameters.
     * @return none.
     */
    void linkControlSetpointPointer(ControlData *controlDataPointer) {_controlSetpoint = controlDataPointer;};

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
     * @param values navigation data pointer.
     * @return none.
     */
    void linkNavigationDataPointer(NavigationData *navigationDataPointer) {_navigationData = navigationDataPointer;};

    /**
     * Returns the kinematics the system needs to achieve the desired
     * guidance inputs.
     *
     * @param values none.
     * @return kinematicSetpoint.
     */
    virtual KinematicData getKinematicOutput() {return _controlOutput;};


protected:

    static ControlData* _controlSetpoint;
    static NavigationData* _navigationData;

    static KinematicData _controlOutput;


private:

    static ControlData _controlSetpointDefault;
    static NavigationData _navigationDataDefault;

    
};





#endif