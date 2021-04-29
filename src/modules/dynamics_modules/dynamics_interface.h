#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"


/**
 * struct used to sent raw actuator commands to a dynamics module
 */
struct ActuatorSetting {
    float actuatorSetting[50];
};



class Dynamics_Interface {
public:

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @return DynamicData.
     */
    virtual DynamicData getDynamicSetpoint() = 0;

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @return DynamicData pointer.
     */
    virtual DynamicData* getDynamicSetpointPointer() = 0;

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
     * @param values ActuatorSetting.
     * @return none.
     */
    virtual void setActuatorsRawData(const ActuatorSetting &actuatorSetpoint) = 0;
    
    
};





#endif