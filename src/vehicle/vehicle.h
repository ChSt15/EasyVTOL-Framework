#ifndef VEHICLE_H
#define VEHICLE_H



#include "Arduino.h"

#include "definitions.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"




class Vehicle: public Guidance, public Control, public Navigation {
public:

    void vehicleThread();

private:

    
};





#endif