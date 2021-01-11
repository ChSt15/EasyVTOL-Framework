#ifndef DEFINITIONS_H 
#define DEFINITIONS_H

//##### Thread Settings #####
#define THREADS_TICK_US 10
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