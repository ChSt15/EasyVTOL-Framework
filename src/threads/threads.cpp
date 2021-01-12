#include "threads/threads.h"


int threadID[7];
volatile uint16_t threadCounter[7];
uint8_t threadUsage[7];
uint8_t cpuUsage = 0;
volatile uint16_t idleThreadCount = 0;
bool threadStartSuccess = false;
void (*func[7])() = {
    thread0,
    thread1,
    thread2,
    thread3,
    thread4,
    thread5,
    thread6
};
uint32_t threadTimeSlice_Ticks[7];
bool threadActive[7] = {
    THREAD0_ACTIVE,
    THREAD1_ACTIVE,
    THREAD2_ACTIVE,
    THREAD3_ACTIVE,
    THREAD4_ACTIVE,
    THREAD5_ACTIVE,
    THREAD6_ACTIVE
};

IntervalControl threadMonitorPrintInterval;


void threadBegin() {

    //For detecting thread start failure
    threadStartSuccess = true;
    
    threads.setMicroTimer(THREADS_TICK_US);

    for (uint8_t i = 0; i < 7; i++) {
        threadTimeSlice_Ticks[i] = (uint8_t)pow(2,(7-i));
    }

    

    //Attempt to start all needed threads and if one fails then set mark threadStartSuccess flag and exit
    for (uint8_t i = 0; i < 7 && threadStartSuccess; i++) {

        if (threadActive[i]) {

            threadID[i] = threads.addThread(func[i]);
            if (threadID[i] == -1) threadStartSuccess = false;
            else threads.setTimeSlice(threadID[i], threadTimeSlice_Ticks[i]);

        }

    }

    threads.setTimeSlice(0, 10);

    //Set default counter values
    for (uint8_t i = 0; i < 7; i++) threadCounter[i] = 0;
    idleThreadCount = 0;

    threadMonitorPrintInterval.setRate(1);

}


void threadControl() {

    if (threadMonitorPrintInterval.isTimeToRun()) {

        threadMonitorPrintInterval.setRate(1);

        uint32_t totalCount = idleThreadCount;
        for (uint8_t i = 0; i < 7; i++) totalCount += threadCounter[i];

        #ifdef PRINT_THREAD_USAGE
            Serial.println("Thread CPU usage:");
        #endif

        for (uint8_t i = 0; i < 7; i++) {

            threadUsage[i] = threadCounter[i]*100/totalCount;
            threadUsage[i] = constrain(threadUsage[i], 0, 100);

            #ifdef PRINT_THREAD_USAGE
                Serial.println("Thread " + String(i) + ": " + String(threadUsage[i]) + "%, ID: " + String(threadID[i]));
            #endif

            cpuUsage = 100 - idleThreadCount*100/totalCount;
            cpuUsage = constrain(cpuUsage, 0, 100);
            threadCounter[i] = 0;

        }

        #ifdef PRINT_THREAD_USAGE
            Serial.println("Total CPU usage: " + String(cpuUsage) + "%");
            Serial.println("Idle Thread: " + String(idleThreadCount*100/totalCount) + "%");
            Serial.println("Thread Start success: " + String(threadStartSuccess));
            Serial.println();
            Serial.println("IMU status: " + deviceStatusToString(IMU::getDeviceStatus()) + ", Rate: " + IMU::getMeasurementRate() + ", Gyro X: " + String(IMU::gyroFifo.shift().x));
            Serial.println("BME status: " + deviceStatusToString(AirData::getDeviceStatus()));
            Serial.println("LED status: " + deviceStatusToString(RGBLED::getDeviceStatus()));
            Serial.println("GPS status: " + deviceStatusToString(GPS::getDeviceStatus()) + ", MeasRate: " + GPS::getMeasurementRate() + ", LoopRate: " + GPS::getRate() +", Sats: " + String(GPS::getSatellites()));
            Serial.println();
        #endif

        idleThreadCount = 0;

    }

    idleThreadCount++;


    /*volatile double cpuWaste = 0;


    for (int i = 0; i < 1000 && !threadMonitorPrintInterval.isTimeToRun(); i++) {
        cpuWaste = sin(cpuWaste*5.4);
    }*/



    //threads.yield();

}



void thread0() {



    

    while(1) {

        IMU::deviceThread();

        threadCounter[0]++;
        threads.yield();

    }

}



void thread1() {



    

    while(1) {

        AirData::deviceThread();
        GPS::deviceThread();
        
        threadCounter[1]++;
        threads.yield();

    }

}



void thread2() {



    

    while(1) {

        //RGBLED::deviceThread();
        
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


