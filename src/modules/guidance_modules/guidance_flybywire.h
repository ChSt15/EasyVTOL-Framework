#ifndef GUIDANCE_FLYBYWIRE_H
#define GUIDANCE_FLYBYWIRE_H



#include "guidance_template.h"

#include "3d_math.h"
#include "interval_control.h"

#include "data_containers/kinematic_data.h"



/**
 * This guidance module is designed to act like a
 * flybywire system. Its a very simple way of
 * transfering manual control to to vehicle.
 */
class GuidanceFlyByWire: public Guidance {
public:

    /**
     * Tells the guidance to rotate vehicle at set rate.
     *
     * @param values rate.
     * @return none.
     */
    void setAngularRate(const Vector &rate) {
        _vehicleControlSettings.angularRate = rate;
    }

    /**
     * Tells the guidance to rotate vehicle to a attitude.
     *
     * @param values attitude.
     * @return none.
     */
    void setAttitude(const Quaternion &attitude) {
        _vehicleControlSettings.attitude = attitude;
    }

    /**
     * Tells the guidance to move vehicle at a set velocity.
     *
     * @param values velocity.
     * @return none.
     */
    void setVelocity(const Vector &velocity) {
        _vehicleControlSettings.velocity = velocity;
    }

    /**
     * Tells the guidance to move vehicle to a set position.
     *
     * @param values position.
     * @return none.
     */
    void setPosition(const Vector &position) {
        _vehicleControlSettings.position = position;
    }

    /**
     * Tells the guidance to stop rotating and moving.
     *
     * @param values none.
     * @return none.
     */
    void stopVehicle() {
        _vehicleControlSettings.angularRate = 0;
        _vehicleControlSettings.velocity = 0;
        _vehicleControlSettings.acceleration = 0;
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
    void setAttitudeControlMode(const CONTROL_MODE &attitudeControlMode) {_vehicleControlSettings.attitudeControlMode = attitudeControlMode;};

    /**
     * Gets the attitude control mode
     *
     * @param values none.
     * @return CONTROL_MODE.
     */
    CONTROL_MODE getAttitudeControlMode() {return _vehicleControlSettings.attitudeControlMode;};

    /**
     * Sets the position control mode
     *
     * @param values positionControlMode.
     * @return none.
     */
    void setPositionControlMode(const CONTROL_MODE &positionControlMode) {_vehicleControlSettings.positionControlMode = positionControlMode;};

    /**
     * Gets the postion control mode
     *
     * @param values none.
     * @return CONTROL_MODE.
     */
    CONTROL_MODE getPositionControlMode() {return _vehicleControlSettings.positionControlMode;};

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

    IntervalControl _interval = IntervalControl(1000); //Loop rate is at 1kHz

};





#endif