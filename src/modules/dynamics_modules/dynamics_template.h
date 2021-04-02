#ifndef DYNAMICS_TEMPLATE_H
#define DYNAMICS_TEMPLATE_H



#include "modules/module_template.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"
#include "data_containers/vehicle_mode.h"


/**
 * struct used to sent raw actuator commands to a dynamics module
 */
struct ActuatorSetting {
    float actuatorSetting[20];
};



class Dynamics: public Module {
public:

    /**
     * Sets the navigation input data from a navigation module.
     * 
     * Use linkNavigationDataPointer() for data linking.
     * 
     * Calling this will remove the link. Data must be linked again!
     *
     * @param values navigation data parameters.
     * @return none.
     */
    void setNavigationInput(const NavigationData &navigationData) {
        _navigationData = &_navigationDataDefault; //Make sure pointer is set to internal data so to not change linked data.
        *_navigationData = navigationData;
    };

    /**
     * Sets the navigation data pointer to the inputed pointer 
     * from a navigation module.
     * 
     * Allows the output module to automatically retrieve its needed
     * data from the pointer.
     * 
     * This must only be called once.
     * 
     * returns false if null pointer was given.
     *
     * @param values navigation data pointer.
     * @return bool.
     */
    bool linkNavigationDataPointer(NavigationData *navigationDataPointer) {
        if (navigationDataPointer == nullptr) return false;
        _navigationData = navigationDataPointer;
        return true;
    };

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual DynamicData getDynamicSetpoint() {return *_dynamicSetpoint;};

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @param values none.
     * @return DynamicData pointer.
     */
    virtual DynamicData* getDynamicSetpointPointer() {return _dynamicSetpoint;};

    /**
     * Sets the dynamics the vehicle needs to achieve
     *
     * @param values none.
     * @return DynamicData.
     */
    virtual void setDynamicSetpoint(const DynamicData &dynamicsSetpoint) {
        *_dynamicSetpoint = dynamicsSetpoint;
    };

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @param values none.
     * @return DynamicData pointer.
     */
    virtual void linkDynamicSetpointPointer(DynamicData* dynamicsSetpoint) {_dynamicSetpoint = dynamicsSetpoint;};

    /**
     * Tells dynamics module to test all actuators. 
     * Giving false will stop testing.
     *
     * @param values bool.
     * @return none.
     */
    virtual void setActuatorTesting(bool testing = true) {
        _actuatorTesting = testing;
    }

    /**
     * Returns if actuators are being tested.
     *
     * @param values none.
     * @return none.
     */
    virtual bool getActuatorTesting() {
        return _actuatorTesting;
    }

    /**
     * Tells dynamics module to enter manual mode
     * Giving false will stop testing.
     *
     * @param values bool.
     * @return none.
     */
    virtual void setActuatorManualMode(bool testing = true) {
        _actuatorManualMode = testing;
    }

    /**
     * Returns if currently in actuator tesing mode.
     *
     * @param values bool.
     * @return none.
     */
    virtual bool getActuatorManualMode() {
        return _actuatorManualMode = false;
    }

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param values ActuatorSetting.
     * @return none.
     */
    virtual void setActuatorsRawData(const ActuatorSetting &actuatorSetpoint) {
        _actuatorManualSetpoint = actuatorSetpoint;
    }


protected:

    //Points to vehicle mode.
    static VEHICLE_MODE *_vehicleMode;

    //Points to dynamics setpoint data container.
    static DynamicData* _dynamicSetpoint;

    //Points to navigation data container.
    static NavigationData* _navigationData;

    //When true then start tesing all actuators. Can be set to true by module when testing is finished.
    bool _actuatorTesting = false;

    //When true then ignore dynamic setpoints and only use manual setpoints
    bool _actuatorManualMode = false;

    //Contains actuator manual raw setpoints. Should only be used when in manual mode.
    ActuatorSetting _actuatorManualSetpoint;


private:

    static DynamicData _dynamicsSetpointDefault;

    static NavigationData _navigationDataDefault;
    
    
};





#endif