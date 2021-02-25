#ifndef STARSHIP_OUTPUT_H
#define STARSHIP_OUTPUT_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "Arduino.h"

#include "outputs/servo_ppm.h"

#include "modules/templates/output_template.h"

#include "data_containers/kinematic_data.h"



class StarshipOutput: public Output {
public:

    class Outputs {
    public:
        PPMChannel TVCServos[4];
        PPMChannel motorCW;
        PPMChannel motorCCW;
    };

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();
    
    /**
     * Inputs the set dynamic data
     *
     * @param values Outputs.
     * @return none.
     */
    void setOutput(const DynamicData &dynamicSetpoint) {_dynamicSetpoint = dynamicSetpoint;}
    

private:

    DynamicData _dynamicSetpoint;
    
    
};





#endif