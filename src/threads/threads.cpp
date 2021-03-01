#include "threads/threads.h"



Scheduler scheduler;

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

Starship vehicle;


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

    threads.setTimeSlice(0, 1);

    //Set default counter values
    for (uint8_t i = 0; i < 7; i++) threadCounter[i] = 0;
    idleThreadCount = 0;

}


void threadSystemMonitor() {

    /*uint32_t totalCount = idleThreadCount;
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
        Serial.println("IMU  status: " + deviceStatusToString(IMU::getDeviceStatus()) + ", Gyro Rate: " + IMU::getGyroRate());
        Serial.println("BME  status: " + deviceStatusToString(AirData::getDeviceStatus()) + ", MeasRate: " + AirData::getMeasurementRate() + ", Temp: " + AirData::temperatureFifo.first());
        Serial.println("LED  status: " + deviceStatusToString(RGBLED::getDeviceStatus()));
        Serial.println("GPS  status: " + deviceStatusToString(GPS::getDeviceStatus()) + ", MeasRate: " + GPS::getMeasurementRate() + ", LoopRate: " + GPS::getRate() +", Sats: " + String(GPS::getSatellites()));
        Serial.println("LORA status: " + deviceStatusToString(LORA_2_4::getDeviceStatus()) + ", LoopRate: " + LORA_2_4::getRate());
        Serial.println("vehicle attitude: w: " + String(vehicle.getAttitude().w) + ", x: " + String(vehicle.getAttitude().x) + ", y: " + String(vehicle.getAttitude().y) + ", z: " + String(vehicle.getAttitude().z));
        Serial.println();
    #endif*/


    KinematicData vehicleKinetics = vehicle.getNavigationData();
    KinematicData vehicleSetpoints = vehicle.getGuidanceData();

    //Serial.println();
    Serial.println("LoopRate: " + String(IMU::getLoopRate()) + ", GyroRate: " + String(IMU::getGyroRate()) + ", AccelRate: " + String(IMU::getAccelRate()) + ", MagRate: " + String(IMU::getMagRate()));
    Serial.println("vehicle attitude: w: " + String(vehicleKinetics.attitude.w) + ", x: " + String(vehicleKinetics.attitude.x) + ", y: " + String(vehicleKinetics.attitude.y) + ", z: " + String(vehicleKinetics.attitude.z));
    //Serial.println("vehicle angularRate: x: " + String(vehicleKinetics.angularRate.x) + ", y: " + String(vehicleKinetics.angularRate.y) + ", z: " + String(vehicleKinetics.angularRate.z));
    //Serial.println("vehicle angularRate setpoint: x: " + String(vehicleSetpoints.angularRate.x) + ", y: " + String(vehicleSetpoints.angularRate.y) + ", z: " + String(vehicleSetpoints.angularRate.z));
    //Serial.println("vehicle attitude setpoint: w: " + String(vehicleSetpoints.attitude.w) + ", x: " + String(vehicleSetpoints.attitude.x) + ", y: " + String(vehicleSetpoints.attitude.y) + ", z: " + String(vehicleSetpoints.attitude.z));
    //Serial.println("IMU Rate: " + String(IMU::getGyroRate()));
    //Serial.println("vehicle accel: x: " + String(vehicle.getAcceleration().x) + ", y: " + String(vehicle.getAcceleration().y) + ", z: " + String(vehicle.getAcceleration().z));
    //Serial.println("vehicle speed: x: " + String(vehicle.getVelocity().x) + ", y: " + String(vehicle.getVelocity().y) + ", z: " + String(vehicle.getVelocity().z));
    //Serial.println("vehicle pos: x: " + String(vehicle.getPosition().x) + ", y: " + String(vehicle.getPosition().y) + ", z: " + String(vehicle.getPosition().z));

    if (Serial.available()) {
        //vehicle.resetInertial();
        while(Serial.available()) Serial.read();
    }

    idleThreadCount = 0;

}


void threadControl() {

    #ifdef DISABLE_MULTITHREADING

        //If threading disabled then go through tasks without multithreading

        static bool firstRun = true;

        if (firstRun) { //Initialise scheduler
            firstRun = false;
            scheduler.attachFunction(tasks0, 64000, TASK_PRIORITY::PRIORITY_REALTIME);
            scheduler.attachFunction(tasks1, 64000, TASK_PRIORITY::PRIORITY_VERYHIGH);
            scheduler.attachFunction(tasks2, 64000, TASK_PRIORITY::PRIORITY_VERYHIGH);
            scheduler.attachFunction(tasks3, 64000, TASK_PRIORITY::PRIORITY_HIGH);
            scheduler.attachFunction(tasks4, 64000, TASK_PRIORITY::PRIORITY_HIGH);
            scheduler.attachFunction(tasks5, 64000, TASK_PRIORITY::PRIORITY_LOW);
            scheduler.attachFunction(tasks6, 64000, TASK_PRIORITY::PRIORITY_LOW);
            scheduler.attachFunction(threadSystemMonitor, 10, TASK_PRIORITY::PRIORITY_NONE);
            scheduler.initialise();
        }

        scheduler.tick();

    #endif


}



//Tasks hold a group of things that need to be done. It is built this way to allow multithreading to be easily disabled

void tasks0() {

    IMU::deviceThread();
    //AirData::deviceThread();
    //LORA_2_4::deviceThread();

}



void tasks1() {

    //GPS::deviceThread();
    vehicle.thread();

}



void tasks2() {

    //vehicle.getNavigationPointer()->thread();
    //RGBLED::deviceThread();

}



void tasks3() {



}



void tasks4() {

    

}



void tasks5() {

    

}



void tasks6() {

    

}





//Each thread hold a task (Group of things) that is then continuesly done

void thread0() {

    while(1) {

        tasks0();

        threadCounter[0]++;

        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread1() {

    while(1) {

        tasks1();
        
        threadCounter[1]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread2() {

    while(1) {

        tasks2();
        
        threadCounter[2]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread3() {
    

    while(1) {
        
        tasks3();
        
        threadCounter[3]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread4() {

    while(1) {

        tasks4();
        
        threadCounter[4]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread5() {

    while(1) {

        tasks5();
        
        threadCounter[5]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}



void thread6() {

    while(1) {

        tasks6();
        
        threadCounter[6]++;
        
        #ifndef DISABLE_THREAD_YIELDING
            threads.yield();
        #endif

    }

}


