#ifndef VEHICLE_H
#define VEHICLE_H



#include "definitions.h"

#include "guidance.h"
#include "navigation.h"
#include "control.h"
#include "kinematics.h"



class Vehicle: public Guidance, public Control, public Navigation, public Kinematics {
public:

    void vehicleThread();

private:

    
};





#endif