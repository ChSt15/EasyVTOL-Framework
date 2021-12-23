#ifndef ACTUATOR_MOUNT_H
#define ACTUATOR_MOUNT_H



#include "actuator_abstract.h"



/**
 * This actuator is to be used to compensate for the force created by the system laying on an object (Table, mount, held in hand etc.) and being tested.
 */
class ActuatorMount: public Actuator_Abstract {
private:




public:

    /**
     * Calculates the actuators force and torqe imparted on system.
     * @returns actuator resulting dynamics.
     */
    const DynamicData& getActuatorDynamicData() const override;

    
};



#endif