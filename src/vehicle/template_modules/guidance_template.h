#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "circular_buffer.h"

#include "vehicle/kinetic_data.h"
#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"


struct PathPoint {

    KineticData pointData;
    uint32_t time = 0;
    bool useTime = false;
    bool isLinear = false;

};


class GuidanceTemplate {
public:

    /**
     * Tells the vehicle to go to a certain point in space.
     *
     * @param values point and time at arrival.
     * @return none.
     */
    //virtual void toPoint(PathPoint state);
    

protected:

    /**
     * Main thread were all calculations and logic
     * are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void guidanceThread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    virtual void guidanceInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);


};





#endif