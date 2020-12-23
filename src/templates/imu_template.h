#ifndef IMU_TEMPLATE_H
#define IMU_TEMPLATE_H
/*
 *
 * This is a template to help with programming devices. This is based off of the imu Sensor.
 * 
 * Thing that NEED to be changed for other devices:
 * - namespace name (here from IMU to <devicename>). This must also be done in imu.cpp
 * - thread name (here from imuThread() to <devicename>Thread())
 * - file names must be changed and therefore the include in the .cpp file
 * - the #ifndef and #define to the <filename> in all caps and a _ for the .
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

//We NEED these includes
#include "Arduino.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



namespace IMUTemplate {

    /*
     * All public methods must be first declared here
    */

    void imuThread();
    
} 





#endif