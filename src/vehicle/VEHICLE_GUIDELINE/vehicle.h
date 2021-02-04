#ifndef VEHICLE_H
#define VEHICLE_H



#include "definitions.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "dynamics.h"



class Vehicle: public Guidance, public Navigation, public Control, public Dynamics {
public:

    void vehicleThread();

private:

    
};





#endif