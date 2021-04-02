#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H



#include "definitions.h"

#include "sensors/imu.h"
#include "sensors/air_data.h"
#include "sensors/gps.h"

#include "comms/lora_2_4.h"

#include "outputs/rgb_led.h"

#include "kraft_kontrol_runner.h"

#include "interval_control.h"



void threadSystemMonitor() {

    static IntervalControl intervalControl(10);

    if (!intervalControl.isTimeToRun()) return; //Only run every 100ms (10Hz)

    if (Serial.available()) {
        

        
    }

}



#endif