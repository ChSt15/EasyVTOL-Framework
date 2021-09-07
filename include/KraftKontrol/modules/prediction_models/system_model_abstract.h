#ifndef SYSTEM_MODEL_ABSTRACT_H
#define SYSTEM_MODEL_ABSTRACT_H



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/modules/actuator_modules/actuator_abstract.h"

#include "KraftKontrol/utils/system_time.h"



/**
 * Used to model system and predict future system state.
 */
class SystemModel_Abstract {
public:

    /**
     * @param systemMass Mass of system in kg.
     * @param angularMass System moment of inertia.
     */
    SystemModel_Abstract(float systemMass, Vector<>& angularMass) {
        systemMass_ = systemMass;
        angularMass_ = angularMass;
    }

    /**
     * Uses system model to predict future system state.
     *
     * @param systemState Current or last state that will be used as start for prediction and will be updated with the new predicted state.
     * @param dTime Delta time to predict. e.g. 100*MILLISECONDS. Must be in nanoseconds.
     * @param iterations Number of iterations. This will split up dTime and run multiple iterations for higher accuracy. Defaults to 1
     */
    void predictState(KinematicData& systemState, const int64_t& dTime_ns, uint32_t iterations = 1) {

        float dTime = (double)dTime_ns/iterations/NANOSECONDS;

        float dTimePow2 = dTime*dTime;
        float dTimeHalfPow2 = dTimePow2*0.5f;
        float dTimeQuartPow4 = dTimePow2*dTimePow2*0.25f;

        DynamicData dynamicState = Actuator_Abstract::getSumDynamics() + getSystemDynamics(systemState);

        for (uint32_t i = 0; i < iterations; i++) {

            //Predict angular stuff
            systemState.angularRate = systemState.angularAcceleration*dTime + systemState.angularRate; //Integrate angular accel to get new angular rate
            systemState.angularRateError = sqrt(systemState.angularAccelerationError*systemState.angularAccelerationError*dTimePow2 + systemState.angularRateError*systemState.angularRateError); //Calculate new error

            Vector<> angleChange = systemState.angularAcceleration*dTimeHalfPow2 + systemState.angularRate*dTime;
            systemState.attitude = Quaternion<>(angleChange, angleChange.magnitude()/2)*systemState.attitude;

            //Predict positional stuff
            systemState.velocity = systemState.velocity + systemState.linearAcceleration*dTime; //Integrate linear accel for velocity
            systemState.velocityError = sqrt(systemState.accelerationError*systemState.accelerationError*dTimePow2 + systemState.velocityError*systemState.velocityError); //Calculate new error

            systemState.position = systemState.position + systemState.velocity*dTime + systemState.linearAcceleration*dTimeHalfPow2; //Integrate linear accel for velocity
            systemState.positionError = sqrt(systemState.accelerationError*systemState.accelerationError*dTimeQuartPow4 + systemState.velocityError*systemState.velocityError*dTimePow2 + systemState.positionError*systemState.positionError); //Calculate new error
            

        }

        //Update time
        systemState.timestamp += dTime_ns;

    }


    /**
     * Calculates using system models the total force and torque of the system without the actuators.
     * Could be implemented to calculate external forces from aerodynamics, bouyancy etc.
     * 
     * @param systemState KinematicData struct containing system state to use to calculate prediction.
     * @returns DynamicData struct with predicted system dynamics.
     */
    virtual DynamicData getSystemDynamics(KinematicData& systemState) {
        return DynamicData(); //Return 0 force and torque when not implemented.
    }

    ///Current mass of the system.
    float systemMass_ = 1;

    ///Current system moment of inertia. Should be a matrix. For simplicity, currently a vector.
    Vector<> angularMass_ = 1;


};



#endif