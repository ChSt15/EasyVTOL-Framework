#ifndef THREADS_H
#define THREADS_H


#include "TeensyThreads.h"

#include "definitions.h"

#include "sensors/imu.h"
#include "sensors/air_data.h"
#include "sensors/gps.h"

#include "comms/lora_2_4.h"

#include "outputs/rgb_led.h"

#include "vehicle/vehicle.h"

#include "utils/interval_control.h"



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

#define DISABLE_MULTITHREADING
//#define DISABLE_THREAD_YIELDING


//##### System Testing #####
#define WAIT_FOR_SERIAL
//#define PRINT_THREAD_USAGE



extern int threadID[7];
extern uint8_t threadUsage[7];
extern uint8_t cpuUsage;
extern bool threadStartSuccess;


void threadBegin();
void threadControl();

void tasks0();
void tasks1();
void tasks2();
void tasks3();
void tasks4();
void tasks5();
void tasks6();
void tasks7();

void thread0();
void thread1();
void thread2();
void thread3();
void thread4();
void thread5();
void thread6();
void thread7();



#endif
