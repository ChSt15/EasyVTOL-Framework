#ifndef OUTPUT_CONTROL_H
#define OUTPUT_CONTROL_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "Arduino.h"

#include "outputs/servo_ppm.h"

#include "vehicle/output_control_template.h"

#include "vehicle/kinetic_data.h"

#include "flight_settings.h"



struct Outputs {
    PPMChannel TVCServos[4];
    PPMChannel motorCW;
    PPMChannel motorCCW;
};



class OutputControl: public OutputControlTemplate {
public:


protected:

    void outputControlThread(const KineticData &vehicleKinetics);
    void outputControlInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);
    
    /**
     * Sets the actuators
     *
     * @param values Outputs.
     * @return none.
     */
    void setActuatorSetpoints(const Outputs &actuatorData) {_actuatorData = actuatorData;}
    

private:

    Outputs _actuatorData;

    FLIGHT_MODE* _flightMode;
    FLIGHT_PROFILE* _flightProfile;
    
    
};





#endif