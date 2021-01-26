#ifndef GENERAL_DYNAMICS_H
#define GENERAL_DYNAMICS_H



#include "Arduino.h"

#include "vehicles/vehicle_dynamics/vehicle_dynamics.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "sensors/imu.h"

#include "utils/device_status.h"
#include "utils/interval_control.h"




class GeneralDynamics: public VehicleDynamics {
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

    IntervalControl interval = IntervalControl(2000);


};





#endif