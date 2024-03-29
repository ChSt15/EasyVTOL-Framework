#ifndef DYNAMICS_ABSTRACT_H
#define DYNAMICS_ABSTRACT_H



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/control_modules/control_interface.h"
#include "KraftKontrol/modules/navigation_modules/navigation_abstract.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "stdint.h"


/**
 * struct used to sent raw actuator commands to a dynamics module
 */
struct ActuatorSetting {
    float actuatorSetting[50];
};


/**
 * Enum for actuator status.
 */
enum eActuatorStatus_t {
    //Actuators diabled and either free to move or in a storage position
    eActuatorStatus_Disabled,
    //Actuators are enabled and in position but wont follow commands.
    eActuatorStatus_Ready,
    //Actuators are enabled and will follow commands
    eActuatorStatus_Enabled
};



class Dynamics_Interface: public Module_Abstract {
public:

    /**
     * Sets the dynamics modules control module.
     * @param controlModule Pointer to module to use.
     */
    virtual void setControlModule(Control_Interface& controlModule) = 0;

    /**
     * Sets the control modules navigation module.
     * @param navigationModule Pointer to module to use.
     */
    virtual void setNavigationModule(Navigation_Abstract& navigationModule) = 0;

    /**
     * Tells dynamics module to test all actuators. 
     * Giving false will stop testing.
     *
     * @param values bool.
     * @return none.
     */
    virtual void setActuatorTesting(bool testing = true) = 0;

    /**
     * Returns if actuators are being tested.
     *
     * @return none.
     */
    virtual bool getActuatorTesting() = 0;

    /**
     * Tells dynamics module to enter manual mode
     * Giving false will stop testing.
     *
     * @param values bool.
     * @return none.
     */
    virtual void setActuatorManualMode(bool testing = true) = 0;

    /**
     * Returns if currently in actuator tesing mode.
     *
     * @param values bool.
     * @return none.
     */
    virtual bool getActuatorManualMode() = 0;

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorSetpoint Actuator Setting.
     * @return none.
     */
    virtual void setActuatorsRawData(const ActuatorSetting &actuatorSetpoint) = 0;

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorSetpoint Actuator Setting.
     * @param actuatorNum actuator to change.
     * @return none.
     */
    virtual void setActuatorsRawData(float actuatorSetpoint, uint16_t actuatorNum) = 0;

    /*
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @returns actuator setting.
     */
    virtual ActuatorSetting getActuatorsRawData() = 0;

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorNum actuator to get.
     * @returns actuator setting.
     */
    virtual float getActuatorsRawData(uint16_t actuatorNum) = 0;

    /**
     * Sets the actuator status. 
     * @param actuatorStatus Actuator mode to set to.
     */
    virtual void setActuatorStatus(eActuatorStatus_t actuatorStatus) = 0;

    /**
     * Checks if module is ready. Should only return true if all actuators are in position and ready to follow commands.
     */
    virtual eActuatorStatus_t getActuatorStatus() = 0;
    
    
};





#endif