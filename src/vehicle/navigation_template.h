#ifndef NAVIGATION_TEMPLATE_H
#define NAVIGATION_TEMPLATE_H



/**
 * This is where the vehicle sensor measurements (sensorfusion) is done. This takes
 * the sensor measurements and calculates all inertial parameters of the vehicle.
 * This allows the navigation to be written in a general way for the simulator.
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/



#include "Arduino.h"

#include "vehicle/kinetic_data.h"
#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"



class NavigationTemplate {
public:

    /**
     * Returns a struct containing all kinetic parameters.
     *
     * @param values none.
     * @return Kinetic paramenters.
     */
    virtual KineticData getNavigationKineticData();


protected:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void navigationThread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    virtual void navigationInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);


private:


};




#endif