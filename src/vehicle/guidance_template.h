#ifndef GUIDANCE_TEMPLATE_H
#define GUIDANCE_TEMPLATE_H



#include "Arduino.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "circular_buffer.h"

#include "vehicle/kinetic_data.h"
#include "flight_settings.h"


struct PathPoint {

    KineticData pointData;
    uint32_t timeAtArrival = 0;
    bool useTime = false;
    bool isLinear = false;

};


class GuidanceTemplate {
public:

    /**
     * Tells to vehicle to go to one point.
     * The path that the vehicle should take might not be 
     * a line. For that use toPointLinear().
     * This program also takes the velocities, and attitudes
     * of the start and end point.
     * Some vehicles might not support (ignore) certain
     * Kinetic parameters. E.g a multicopter due to it being 
     * underactuated will ignore attitude parameters. 
     * 
     * The time at arrival is the time at which the vehicle 
     * should reach the point relative to the last point or 
     * in the case that this has already happend then relative
     * to the time the function is called and is in microseconds.
     *
     * @param values point and time at arrival.
     * @return none.
     */
    void toPoint(KineticData state, uint32_t timeAtArrivalus);
    

protected:

    virtual void guidanceThread();
    virtual void guidanceInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);


};





#endif