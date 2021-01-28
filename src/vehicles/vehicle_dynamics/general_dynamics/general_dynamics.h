#ifndef GENERAL_DYNAMICS_H
#define GENERAL_DYNAMICS_H



#include "Arduino.h"

#include "vehicles/vehicle_dynamics/vehicle_dynamics.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "sensors/imu.h"

#include "utils/device_status.h"
#include "utils/interval_control.h"


#define LOOP_RATE_LIMIT 2000


class GeneralDynamics: public VehicleDynamics {
public:

    void resetInertialTEST() {_position = Vector(0,0,0); _velocity = Vector(0,0,0);}; //To be removed or replaced. This is only for testing Inertial navigation


protected:

    void sensorFusionThread();


private:

    uint32_t lastGyroTimestamp = 0;
    uint32_t lastAccelTimestamp = 0;
    uint32_t lastMagTimestamp = 0;

    Vector gyroAverage;

    bool gyroInitialized = false;
    bool accelInitialized = false;
    bool magInitialized = false;

    IntervalControl interval = IntervalControl(LOOP_RATE_LIMIT);


};





#endif