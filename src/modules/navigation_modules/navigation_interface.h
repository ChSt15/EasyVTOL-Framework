#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "data_containers/navigation_data.h"



class Navigation_Interface {
public:

    /**
     * Returns a struct containing all the vehicles
     * current navigation parameters.
     *
     * @return navigation paramenters.
     */
    virtual NavigationData getNavigationData() = 0;

    /**
     * Returns a pointer to a struct containing all 
     * the vehicles current kinematic parameters.
     * 
     * This is usefull for data linking instead of 
     * always having to pass data manually.
     *
     * @return kinematic parameter pointer.
     */
    virtual NavigationData* getNavigationDataPointer() = 0;

    /**
     * Returns the current position accuracy in meters.
     * Returns -1.0 if unsupported.
     * 
     * @return float.
     */
    virtual float getPositionAccuracy() {return -1.0f;}

    /**
     * Returns the current attitude accuracy in radians.
     * Returns -1.0 if unsupported.
     * 
     * @return float.
     */
    virtual float getAttitudeAccuracy() {return -1.0f;}

};




#endif