#ifndef VEHICLE_H
#define VEHICLE_H



#include "Arduino.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "utils/device_status.h"




class Vehicle: public Guidance, public Control, public Navigation {
public:

    void vehicleThread();

private:

    
};





#endif