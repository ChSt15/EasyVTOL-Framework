#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"
#include "data_containers/vehicle_mode.h"



class Dynamics: public Module {
public:

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

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual DynamicData getDynamicSetpoint() {return *_dynamicSetpoint;};

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @param values none.
     * @return DynamicData pointer.
     */
    virtual DynamicData* getDynamicSetpointPointer() {return _dynamicSetpoint;};

    /**
     * Sets the dynamics the vehicle needs to achieve
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual void setDynamicSetpoint(const DynamicData &dynamicsSetpoint) {
        *_dynamicSetpoint = dynamicsSetpoint;
    };

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @param values none.
     * @return DynamicData pointer.
     */
    virtual void linkDynamicSetpointPointer(DynamicData* dynamicsSetpoint) {_dynamicSetpoint = dynamicsSetpoint;};


protected:

    static VEHICLE_MODE *_vehicleMode;

    static DynamicData* _dynamicSetpoint;

    static NavigationData* _navigationData;


private:

    static DynamicData _dynamicsSetpointDefault;

    static NavigationData _navigationDataDefault;
    
    
};





#endif