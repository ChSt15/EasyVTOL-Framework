#ifndef DYNAMICS_H
#define DYNAMICS_H


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

#include "flight_modes.h"



class Dynamics {
public:



protected:

    void dynamicsThread(KineticData vehicleKinetics);
    void dynamicsInit(FLIGHT_MODE* flightModePointer);
    
    /**
     * Sets the dynamic data. This should all be in local coordinate system
     *
     * @param values dynamics.
     * @return none.
     */
    void setDynamicsSetpoint(DynamicData dynamics) {_dynamicData = dynamics;}
    

private:

    DynamicData _dynamicData;

    FLIGHT_MODE* _flightMode;
    
    
};





#endif