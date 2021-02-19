#ifndef OUTPUT_CONTROL_TEMPLATE_H
#define OUTPUT_CONTROL_TEMPLATE_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "Arduino.h"

#include "vehicle/kinetic_data.h"

#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"



class OutputControlTemplate {
public:


protected:

    virtual void outputControlThread(const KineticData &vehicleKinetics);
    virtual void outputControlInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);
    
    
};





#endif