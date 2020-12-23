#ifndef OUTPUTs_MAIN_H
#define OUTPUT_MAIN_H
/*
 *
 * This is where all output related stuff should be.
 * Meaning this is where all output run methods should be 
 * and any extra processes that are related to output.
 * 
 * We will use namespaces to declare global variables but
 * at the same time reduce overall clutter when programming.
 * 
*/


#include "Arduino.h"

#include "TeensyThreads.h"

#include "utils/interval_control.h"

#include "definitions.h"



namespace Output {

    extern volatile uint32_t testData;

    void outputThread(void* arg);
    
} 





#endif