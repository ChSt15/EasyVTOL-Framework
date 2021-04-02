#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "modules/module_template.h"

#include "data_containers/navigation_data.h"



class Navigation: public Module {
public:

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     * If only the kinematic parameters are needed
     * then use getKinematicData().
     *
     * @param values none.
     * @return navigation paramenters.
     */
    virtual NavigationData getNavigationData() {return _navigationData;};

    /**
     * Returns a struct containing all the vehicles
     * current kinematic parameters.
     *
     * @param values none.
     * @return kinematic paramenters.
     */
    virtual KinematicData getKinematicData() {return _navigationData;};

    /**
     * Returns the current position accuracy in meters.
     * Returns -1.0 if unsupported.
     * 
     * @param values none.
     * @return float.
     */
    virtual float getPositionAccuracy() {return -1.0f;}

    /**
     * Returns the current attitude accuracy in radians.
     * Returns -1.0 if unsupported.
     * 
     * @param values none.
     * @return float.
     */
    virtual float getAttitudeAccuracy() {return -1.0f;}

    /**
     * Returns a pointer to a struct containing all 
     * the vehicles current kinematic parameters.
     * 
     * This is usefull for data linking instead of 
     * always having to pass data manually.
     *
     * @param values none.
     * @return kinematic parameter pointer.
     */
    virtual NavigationData* getNavigationDataPointer() {return &_navigationData;};


protected:

    /**
     * This pointer points to the global navigation data kinematic data container
     */
    static NavigationData _navigationData;

};




#endif