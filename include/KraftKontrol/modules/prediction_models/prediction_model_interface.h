#ifndef PREDICTION_MODEL_INTERFACE_H
#define PREDICTION_MODEL_INTERFACE_H



#include "KraftKontrol/data_containers/navigation_data.h"
#include "KraftKontrol/data_containers/dynamic_data.h"

#include "KraftKontrol/utils/system_time.h"



class PredictionModel_Interface {
public:

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @param lastState Current or last state that will be used as start for prediction and will be updated with the new predicted state.
     * @param dTime Delta time to predict. e.g. 100*MILLISECONDS. Must be in nanoseconds.
     * @param iterations Number of iterations. This will split up dTime and run multiple iterations for higher linearity with rotations.
     */
    virtual void predictState(KinematicData& lastState, const int64_t& dTime_ns, uint32_t iterations = 100) = 0;
    
    
};



/**
 * Use this model when model is unknown. This does a simple prediction.
 */
class General_PredictionModel: public PredictionModel_Interface {
public:

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @param lastState Current or last state that will be used as start for prediction and will be updated with the new predicted state.
     * @param dTime Delta time to predict. e.g. 100*MILLISECONDS. Must be in nanoseconds.
     * @param iterations Number of iterations. This will split up dTime and run multiple iterations for higher linearity with rotations.
     */
    void predictState(KinematicData& lastState, const int64_t& dTime_ns, uint32_t iterations = 10) override {

        float dTime = dTime_ns/iterations/NANOSECONDS;

        float dTimePow2 = dTime*dTime;
        float dTimeHalfPow2 = dTimePow2*0.5f;
        float dTimeQuartPow4 = dTimePow2*dTimePow2*0.25f;

        for (uint32_t i = 0; i < iterations; i++) {

            //Predict angular stuff
            lastState.angularRate = lastState.angularAcceleration*dTime + lastState.angularRate; //Integrate angular accel to get new angular rate
            lastState.angularRateError = sqrt(lastState.angularAccelerationError*lastState.angularAccelerationError*dTimePow2 + lastState.angularRateError*lastState.angularRateError); //Calculate new error

            Vector<> angleChange = lastState.angularAcceleration*dTimeHalfPow2 + lastState.angularRate*dTime;
            lastState.attitude = Quaternion<>(angleChange, angleChange.magnitude()/2)*lastState.attitude;

            //Predict positional stuff
            lastState.velocity = lastState.velocity + lastState.linearAcceleration*dTime; //Integrate linear accel for velocity
            lastState.velocityError = sqrt(lastState.accelerationError*lastState.accelerationError*dTimePow2 + lastState.velocityError*lastState.velocityError); //Calculate new error

            lastState.position = lastState.position + lastState.velocity*dTime + lastState.linearAcceleration*dTimeHalfPow2; //Integrate linear accel for velocity
            lastState.positionError = sqrt(lastState.accelerationError*lastState.accelerationError*dTimeQuartPow4 + lastState.velocityError*lastState.velocityError*dTimePow2 + lastState.positionError*lastState.positionError); //Calculate new error

            

        }

        //Update time
        lastState.timestamp += dTime_ns;

    }

};



#endif