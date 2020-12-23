#ifndef SENSORS_MAIN_H
#define SENSORS_MAIN_H
/*
 *
 * This is where all sensor related stuff should be.
 * Meaning this is where all sensors run methods should be 
 * and any extra processes that are related to sensors.
 * 
 * We will use namespaces to declare global variables but
 * at the same time reduce overall clutter when programming.
 * 
*/


#include "Arduino.h"

#include "TeensyThreads.h"

#include "utils/interval_control.h"

#include "definitions.h"



namespace Sensors {

    extern volatile uint32_t testData;

    void sensorThread(void* arg);
    
} 





#endif