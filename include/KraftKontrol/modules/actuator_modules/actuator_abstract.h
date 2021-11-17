#ifndef ACTUATOR_ABSTRACT_H
#define ACTUATOR_ABSTRACT_H



#include "Arduino.h"

#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "KraftKontrol/utils/list.h"

#include "KraftKontrol/platforms/platform_hal.h"



/**
 * Class to be inhereted from. 
 * Supports calculating total system force and torque for state estimation and simulation.
 * 
 */
class Actuator_Abstract {
private:

    ///List of all actuators that exist.
    static List<Actuator_Abstract*>& actuatorList();


protected:

    /**
     * Constructor called by subclasses-
     * @param addToList If true then the actuator will be added to list of all actuators and included in calculations.
     */
    Actuator_Abstract() {
        actuatorList().append(this);
    }


public:
    
    /**
     * Virtual destructor
     * Removes actuator from list
     */
    virtual ~Actuator_Abstract() {
        actuatorList().removeAllEqual(this);
    }

    /**
     * Calculates the actuators force and torqe imparted on system.
     * @returns actuator resulting dynamics.
     */
    virtual const DynamicData& getActuatorDynamicData() const = 0;

    /**
     * Calculates total resulting system dynamics from all actuators in local coordinate system.
     * @returns total system force/torque.
     */
    static DynamicData getSumDynamics() {

        DynamicData totalDynamics;

        for (uint32_t i = 0; i < actuatorList().getNumItems(); i++) {

            totalDynamics += actuatorList()[i]->getActuatorDynamicData();

        }

        return totalDynamics;

    }

    
};



#endif