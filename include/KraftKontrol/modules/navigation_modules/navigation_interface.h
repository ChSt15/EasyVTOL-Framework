#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "KraftKontrol/data_containers/navigation_data.h"



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
     * Sets the home position.
     * All position data will be in refernce to this home position.
     *
     * @param homePosition Is the position to be used as home.
     */
    virtual void setHome(WorldPosition homePosition) = 0;

};




#endif