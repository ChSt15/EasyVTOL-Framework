#include "threads/threads.h"


int threadID[8];
volatile uint16_t threadCounter[8];
uint8_t threadUsage[8];
uint8_t cpuUsage = 0;
volatile uint16_t idleThreadCount = 0;
bool threadStartSuccess = false;
void (*func[8])() = {
    thread0,
    thread1,
    thread2,
    thread3,
    thread4,
    thread5,
    thread6,
    thread7,
};
bool threadActive[8] = {
    THREAD0_ACTIVE,
    THREAD1_ACTIVE,
    THREAD2_ACTIVE,
    THREAD3_ACTIVE,
    THREAD4_ACTIVE,
    THREAD5_ACTIVE,
    THREAD6_ACTIVE,
    THREAD7_ACTIVE
};

IntervalControl threadMonitorPrintInterval;


void threadBegin() {

    //For detecting thread start failure
    threadStartSuccess = true;
    
    threads.setSliceMicros(TIMESLICE_US);

    //Attempt to start all needed threads and if one fails then set mark threadStartSuccess flag and exit
    for (uint8_t i = 0; i < 8 && threadStartSuccess; i++) {

        if (threadActive[i]) {

            threadID[i] = threads.addThread(func[i]);
            if (threadID[i] == -1) threadStartSuccess = false;

        }

    }

    //Set default counter values
    for (uint8_t i = 0; i < 8; i++) threadCounter[i] = 0;
    idleThreadCount = 0;

    threadMonitorPrintInterval.setRate(1);

}


void threadControl() {

    if (threadMonitorPrintInterval.isTimeToRun()) {

        uint32_t totalCount = idleThreadCount;
        for (uint8_t i = 0; i < 8; i++) totalCount += threadCounter[i];

        #ifdef PRINT_THREAD_USAGE
            Serial.println("Thread CPU usage:");
        #endif

        for (uint8_t i = 0; i < 8; i++) {

            threadUsage[i] = threadCounter[i]*100/totalCount;
            threadUsage[i] = constrain(threadUsage[i], 0, 100);

            #ifdef PRINT_THREAD_USAGE
                Serial.println("Thread " + String(i) + ": " + String(threadUsage[i]) + "%");
            #endif

            cpuUsage = 100 - idleThreadCount*100/totalCount;
            cpuUsage = constrain(cpuUsage, 0, 100);

        }

        #ifdef PRINT_THREAD_USAGE
            Serial.println("Total CPU usage: " + String(cpuUsage) + "%");
            Serial.println("Idle Thread: " + String(idleThreadCount*100/totalCount) + "%");
            Serial.println();
        #endif

    }

    idleThreadCount++;

}



void thread0() {



    

    while(1) {



        threadCounter[0]++;
        threads.yield();

    }

}



void thread1() {



    

    while(1) {

        
        threadCounter[1]++;
        threads.yield();

    }

}



void thread2() {



    

    while(1) {

        
        threadCounter[2]++;
        threads.yield();

    }

}



void thread3() {



    

    while(1) {

        
        threadCounter[3]++;
        threads.yield();

    }

}



void thread4() {



    

    while(1) {

        
        threadCounter[4]++;
        threads.yield();

    }

}



void thread5() {



    

    while(1) {

        
        threadCounter[5]++;
        threads.yield();

    }

}



void thread6() {



    

    while(1) {

        
        threadCounter[6]++;
        threads.yield();

    }

}



void thread7() {



    

    while(1) {

        
        threadCounter[7]++;
        threads.yield();

    }

}











