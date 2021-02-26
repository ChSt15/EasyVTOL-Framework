#ifndef OUTPUT_TEMPLATE_H
#define OUTPUT_TEMPLATE_H


/**
 * This is where the vehicle takes the dynamics setpoint and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/



#include "modules/templates/module_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"

#include "utils/interval_control.h"


class Output: public Module {
public:

    /**
     * Sets the dynamics the vehicle must have.
     * 
     * Use linkDynamicSetpointPointer() for data linking.
     * 
     * Calling this will remove the link. Data must be linked again!
     *
     * @param values dynamicSetpoint.
     * @return none.
     */
    void dynamicSetpoint(const DynamicData &dynamicSetpoint) {
        _dynamicSetpoint = &_dynamicSetpointDefault; //Make sure pointer is set to internal data so to not change linked data.
        *_dynamicSetpoint = dynamicSetpoint;
    };

    /**
     * Sets the dynamics data pointer to the inputed pointer 
     * from a navigation module.
     * 
     * Allows the output module to automatically retrieve its needed
     * data from the pointer.
     * 
     * This must only be called once.
     * 
     * returns false if null pointer was given.
     *
     * @param values dynamics data pointer.
     * @return bool.
     */
    bool linkDynamicSetpointPointer(DynamicData *dynamicSetpointPointer) {
        if (dynamicSetpointPointer == nullptr) return false;
        _dynamicSetpoint = dynamicSetpointPointer;
        return true;
    };
    	
    /**
     * Disables outputs of actuators.
     * Basicaly disarms vehicle.
     *
     * @param values none.
     * @return none.
     */
    void disableOutput() {
        outputEnabled = false;
    };
    
    /**
     * Enables outputs of actuators.
     * Basicaly arms vehicle.
     *
     * @param values none.
     * @return none.
     */
    void enableOutput() {
        outputEnabled = true;
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
     * Allows the output module to automatically retrieve its needed
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


    

protected:

    bool outputEnabled = false;

    IntervalControl _interval = IntervalControl(1000);

    static DynamicData* _dynamicSetpoint;
    static DynamicData _dynamicSetpointDefault;

    static NavigationData* _navigationData;
    static NavigationData _navigationDataDefault;
    
    
};





#endif