#ifndef GENERAL_DYNAMICS_H
#define GENERAL_DYNAMICS_H



#include "Arduino.h"

#include "vehicles/vehicle_dynamics/vehicle_dynamics.h"

#include "quaternion_math.h"
#include "vector_math.h"

#include "sensors/imu.h"

#include "utils/device_status.h"




class GeneralDynamics: public VehicleDynamics {
protected:

    void sensorFusionThread();

private:


};





#endif