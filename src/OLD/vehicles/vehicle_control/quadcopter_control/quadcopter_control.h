#ifndef QUADCOPTER_CONTROL_H
#define QUADCOPTER_CONTROL_H



#include "Arduino.h"

#include "vehicles/vehicle_control/vehicle_control.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"



class QuadcopterControl: public VehicleControl {
protected:

    void controlThread();

};




#endif