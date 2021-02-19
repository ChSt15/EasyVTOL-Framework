#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H


/**
 * This is where the vehicle "output mapping" is done. This takes the outputs of the control
 * and transforms (post processes) it to the different servo outputs, motor ESCs etc.
 * This allows the control to be written in a very general way and streamlines the 
 * development if the control systems. 
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
 * 
 * Dynamics does not require to be a closed loop solution. Meaning it could simply map the control
 * outputs to the vehicle controls (flaps, motors etc.) and the control algorithms can to the rest
 * but implementing closed loop control allows for more accuracy and further linearity of control 
 * outputs and simplifies the control algorithms. 
*/


#include "Arduino.h"

#include "vehicle/kinetic_data.h"
#include "vehicle/dynamic_data.h"
#include "vehicle/flight_modes.h"
#include "vehicle/flight_profiles.h"



class DynamicsTemplate {
public:



protected:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void dynamicsThread(KineticData vehicleKinetics);

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    virtual void dynamicsInit(FLIGHT_MODE* flightModePointer, FLIGHT_PROFILE* flightProfilePointer);
    
    /**
     * Sets the dynamic data. This should all be in local coordinate system
     *
     * @param values dynamics.
     * @return none.
     */
    virtual void setDynamicsSetpoint(DynamicData dynamics);
    

private:
    
    
};





#endif