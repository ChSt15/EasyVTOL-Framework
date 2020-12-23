#ifndef DEFINITIONS_H 
#define DEFINITIONS_H

//##### Thread Settings #####
#define TIMESLICE_US 125
#define THREAD0_ACTIVE true
#define THREAD1_ACTIVE false
#define THREAD2_ACTIVE false
#define THREAD3_ACTIVE false
#define THREAD4_ACTIVE false
#define THREAD5_ACTIVE false
#define THREAD6_ACTIVE false
#define THREAD7_ACTIVE false

//##### System Testing #####
#define YIELD_THREADS_WHEN_DONE
#define DO_BIG_CALC_ON_THREADS

#define WAIT_FOR_SERIAL

#define PRINT_THREAD_USAGE


//##### Loop Rates #####
#define CONTROL_RATE_HZ 4000
#define IMU_RATE_HZ 8000

//##### Other #####
#define USB_BAUD_RATE 115200













#endif