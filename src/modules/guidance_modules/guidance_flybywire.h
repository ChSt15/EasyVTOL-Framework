#ifndef GUIDANCE_FLYBYWIRE_H
#define GUIDANCE_FLYBYWIRE_H



#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "guidance_interface.h"

#include "modules/module_abstract.h"

#include "data_containers/kinematic_data.h"



/**
 * This guidance module is designed to act like a
 * flybywire system. Its a very simple way of
 * transfering manual control to to vehicle.
 */
class GuidanceFlyByWire: public Guidance_Interface, Task_Abstract {
public:

    /**
     * Creates a module based class and automatically adds it to the scheduler.
     * 
     * @param rate is the rate at which it will be ran at.
     * @param priority is the priority the module will have.
     */
    GuidanceFlyByWire() : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_High, true) {}

    /**
     * Tells the guidance to rotate vehicle at set rate.
     *
     * @param values rate.
     * @return none.
     */
    void setAngularRate(const Vector<> &rate) {
        vehicleControlSettings_.angularRate = rate;
    }

    /**
     * Tells the guidance to rotate vehicle to a attitude.
     *
     * @param values attitude.
     * @return none.
     */
    void setAttitude(const Quaternion<> &attitude) {
        vehicleControlSettings_.attitude = attitude;
    }

    /**
     * Tells the guidance to move vehicle at a set velocity.
     *
     * @param values velocity.
     * @return none.
     */
    void setVelocity(const Vector<> &velocity) {
        vehicleControlSettings_.velocity = velocity;
    }

    /**
     * Tells the guidance to move vehicle to a set position.
     *
     * @param values position.
     * @return none.
     */
    void setPosition(const Vector<> &position) {
        vehicleControlSettings_.position = position;
    }

    /**
     * Tells the guidance to stop rotating and moving.
     *
     * @param values none.
     * @return none.
     */
    void stopVehicle() {
        vehicleControlSettings_.angularRate = 0;
        vehicleControlSettings_.velocity = 0;
        vehicleControlSettings_.acceleration = 0;
    }

    /**
     * Returns distance from endpoint. Used for switching between guidance modules.
     *
     * @param values none.
     * @return float.
     */
    float distanceFromEndpoint() {return 0.0;};

    /**
     * Sets the attitude control mode
     *
     * @param values attitudeControlMode.
     * @return none.
     */
    void setAttitudeControlMode(const eControlMode_t &attitudeControlMode) {vehicleControlSettings_.attitudeControlMode = attitudeControlMode;};

    /**
     * Gets the attitude control mode
     *
     * @param values none.
     * @return eControlMode_t.
     */
    eControlMode_t getAttitudeControlMode() {return vehicleControlSettings_.attitudeControlMode;};

    /**
     * Sets the position control mode
     *
     * @param values positionControlMode.
     * @return none.
     */
    void setPositionControlMode(const eControlMode_t &positionControlMode) {vehicleControlSettings_.positionControlMode = positionControlMode;};

    /**
     * Gets the postion control mode
     *
     * @param values none.
     * @return eControlMode_t.
     */
    eControlMode_t getPositionControlMode() {return vehicleControlSettings_.positionControlMode;};

    /**
     * Returns a struct containing all the vehicles
     * setpoint data that feeds to the controller.
     *
     * @return control parameters.
     */
    virtual ControlData getControlSetpoint() {return vehicleControlSettings_;}

    /**
     * Returns a pointer to a struct containing all the 
     * vehicles setpoint data that feeds to the controller.
     * 
     * Using pointers allows for data linking instead of
     * always passing data.
     *
     * @return control parameter pointer.
     */
    virtual ControlData* getControlSetpointPointer() {return &vehicleControlSettings_;}

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();



private:

    ControlData vehicleControlSettings_;

    uint32_t _lastRunTimestamp = 0;

    bool initialised = false;


};





#endif