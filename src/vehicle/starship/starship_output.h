#ifndef STARSHIP_OUTPUT_H
#define STARSHIP_OUTPUT_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "definitions.h"

#include "utils/interval_control.h"

#include "outputs/servo_ppm.h"

#include "modules/output_modules/output_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"



class StarshipOutput: public Output {
public:

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
    

private:

    //TVC servos. 0 is x+, 1 is y+, 2 is x-, 3 is y-.
    PPMChannel TVCServos[4];
    //CW motor control.
    PPMChannel motorCW;
    //CCW motor control.
    PPMChannel motorCCW;
    //Up Left flap servo.
    PPMChannel flapUL;
    //Up Right flap servo.
    PPMChannel flapUR;
    //Down Left flap servo.
    PPMChannel flapDL;
    //Down Right flap servo.
    PPMChannel flapDR;
    
    
};





#endif