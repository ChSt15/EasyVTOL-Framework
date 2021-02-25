#ifndef OUTPUT_TEMPLATE_H
#define OUTPUT_TEMPLATE_H


/**
 * This is where the vehicle takes the dynamics setpoint and controls the actuators. 
 * Because this class can be augmented by the simulator, this structure also allows for very
 * easy testing of the vehicle code in the simulator in a "drag and drop" type way.
*/



#include "module_template.h"

#include "data_containers/dynamic_data.h"



class Output: public Module {
public:

    virtual void setOutput(const DynamicData &dynamicSetpoint) = 0;
    
    
};





#endif