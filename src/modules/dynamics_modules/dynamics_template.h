#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "modules/templates/module_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"



class Dynamics: public Module {
public:

    /**
     * Sets the kinetic setpoints from which the needed forces
     * are calculated.
     *
     * @param values kinematicSetpoint.
     * @return none.
     */
    void setKinematicSetpoint(const KinematicData &kinematicSetpoint) {
        _kinematicSetpoint = &_kinematicSetpointDefault;
        *_kinematicSetpoint = kinematicSetpoint;
    };

    /**
     * Sets the kinetic setpoints pointer to the inputed one.
     * 
     * This allows for data linking meaning that the data does 
     * not need to be passed between modules.
     * 
     * Returns false if linking failed. Can only happen if pointer was
     * null pointer.
     *
     * @param values kinematicSetpoint.
     * @return bool.
     */
    bool linkKinematicSetpointPointer(KinematicData* kinematicSetpointPointer) {
        if (kinematicSetpointPointer == nullptr) return false;
        _kinematicSetpoint = kinematicSetpointPointer;
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
    virtual DynamicData getDynamicSetpoint() {_dynamicOutput;};

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
    virtual DynamicData* getDynamicSetpointPointer() {&_dynamicOutput;};


protected:

    static KinematicData* _kinematicSetpoint;

    static DynamicData _dynamicOutput;

    static NavigationData* _navigationData;


private:

    static KinematicData _kinematicSetpointDefault;

    static NavigationData _navigationDataDefault;
    
    
};





#endif