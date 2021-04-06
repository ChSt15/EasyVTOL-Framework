#ifndef KRAFTKOMMUNICATION_H
#define KRAFTKOMMUNICATION_H



#include "SX12XX-LoRa-master/src/SX128XLT.h"



struct KraftPacket {
    const uint8_t packetStart1 = 0xAA;
    const uint8_t packetStart2 = 0xFF;
    const uint8_t packetEnd = 0xFA;
};



#endif 
