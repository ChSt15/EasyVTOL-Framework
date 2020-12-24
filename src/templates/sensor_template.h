#ifndef SENSOR_TEMPLATE_H
#define SENSOR_TEMPLATE_H
/*
 *
 * This is a template to help with programming devices. This is based off of the imu Sensor.
 * 
 * ################## IMPORTANT!!!! #################
 * Things that NEED to be changed for other devices:
 * - namespace name (here from SensorTemplate to <devicename>). This must also be done in sensor_template.cpp
 * - thread name (here from imuThread() to <devicename>Thread()). This must also be done in sensor_template.cpp
 * - file names must be changed and therefore the include in the .cpp file
 * - the #ifndef and #define to the <filename> in all caps and a _ for the (from SENSOR_TEMPLATE_H to <DEVICENAME>_H)
 * 
 * We will use namespaces to declare global variables but
 * at the same time reduce overall clutter when programming other 
 * systems that rely on this device.
 * By doing this we use less stack and heap that normal OOP (object 
 * oriented programming) would use because we dont need OOP
 * for hardware things that we only have one of (e.g. radio module, IMU).
 * But device like motors or servos should use OOP because we generally
 * have many of those.
 * 
*/

//We NEED these includes. But other includes can be added
#include "Arduino.h"

#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



namespace SensorTemplate {

    //All methods and variables declared here are public

    extern CircularBuffer <float, 50> sensorDataFifo; 
    extern CircularBuffer <uint32_t, 50> timestampFifo;

    uint16_t getDataCount();

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif