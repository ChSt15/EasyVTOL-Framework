#ifndef DYNAMICS_H
#define DYNAMICS_H


/**
 * This is where the vehicle "output mapping" is done. This takes the outputs of the control
 * and transforms (post processes) it to the different servo outputs, motor ESCs etc.
 * This allows the control to be written in a very general way and streamlines the 
 * development if the control systems. 
 * Because this class will be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a very "drag and drop" type way.
*/


#include "Arduino.h"

#include "vehicle/kinetic_data.h"
#include "vehicle/dynamic_data.h"


class Dynamics {
public:

    void setDynamics(DynamicData dynamics) {_dynamicData = dynamics;}


protected:

    void dynamicsThread(KineticData vehicleKinetics);
    

private:

    DynamicData _dynamicData;

    
};





#endif