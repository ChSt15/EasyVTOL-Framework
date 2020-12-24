#ifndef THREADS_H
#define THREADS_H


#include "TeensyThreads.h"

#include "definitions.h"

#include "sensors/imu.h"
#include "sensors/air_data.h"

#include "utils/interval_control.h"


extern int threadID[7];
extern uint8_t threadUsage[7];
extern uint8_t cpuUsage;
extern bool threadStartSuccess;


void threadBegin();
void threadControl();

void thread0();
void thread1();
void thread2();
void thread3();
void thread4();
void thread5();
void thread6();
void thread7();



#endif
