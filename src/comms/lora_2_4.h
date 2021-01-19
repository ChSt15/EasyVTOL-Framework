#ifndef LORA_2_4_H
#define LORA_2_4_H

#include "Arduino.h"


#include "SX128XLT.h"
#include "vector_math.h"
#include "quaternion_math.h"
#include "CircularBuffer.h"

#include "utils/interval_control.h"
#include "utils/device_status.h"

#include "definitions.h"



#define COMMS_RATE 100


#define LORA_FREQUENCY 2445000000L
#define LORA_BANDWIDTH LORA_BW_0800
#define LORA_SPREADINGFACTOR LORA_SF12
#define LORA_CODERATE LORA_CR_4_6
#define LORA_POWER_dB 10 //This should be kept at <=10dB for legal reasons


namespace LORA_2_4 {

    extern CircularBuffer <Quaternion, 100> attitudeSendFifo;

    void deviceThread();

    uint32_t getRate();

    DeviceStatus getDeviceStatus();
    
} 





#endif