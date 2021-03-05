#ifndef KWAD_DYNAMICS_H
#define KWAD_DYNAMICS_H


/**
 * This is where the vehicle takes the dynamics outputs and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/


#include "definitions.h"

#include "utils/interval_control.h"

#include "outputs/servo_ppm.h"

#include "modules/dynamics_modules/dynamics_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"



class KwadDynamics: public Dynamics {
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

    //Motor Front left
    PPMChannel motorFL;
    //Motor Front right
    PPMChannel motorFR;
    //Motor Back left
    PPMChannel motorBL;
    //Motor Back Right
    PPMChannel motorBR;
    
    
};





#endif