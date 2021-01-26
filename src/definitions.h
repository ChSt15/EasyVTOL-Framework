#ifndef DEFINITIONS_H 
#define DEFINITIONS_H



//General includes all files should have
#include "boards/board_v_1_0.h" //Board version for pins etc.


//##### Thread Settings #####
#define THREADS_TICK_US 30 //So small so that IMU can be read at 32kHz (32.5us)

#define THREAD0_ACTIVE true
#define THREAD1_ACTIVE true
#define THREAD2_ACTIVE true
#define THREAD3_ACTIVE true
#define THREAD4_ACTIVE true
#define THREAD5_ACTIVE true
#define THREAD6_ACTIVE true

#define THREAD0_TIMESLICE_TICKS 1
#define THREAD1_TIMESLICE_TICKS 1
#define THREAD2_TIMESLICE_TICKS 1
#define THREAD3_TIMESLICE_TICKS 1
#define THREAD4_TIMESLICE_TICKS 1
#define THREAD5_TIMESLICE_TICKS 1
#define THREAD6_TIMESLICE_TICKS 1

//#define DISABLE_MULTITHREADING
//#define DISABLE_THREAD_YIELDING


//##### System Testing #####
#define WAIT_FOR_SERIAL
#define PRINT_THREAD_USAGE


//##### Loop Rates #####
#define CONTROL_RATE_HZ 4000


//##### Safe Guards #####
#define SENSOR_MEASUREMENT_TIMEOUT_US 200000L


//##### Other #####
#define USB_BAUD_RATE 115200














#endif