#ifndef DYNAMIC_DATA_H
#define DYNAMIC_DATA_H



#include "3d_math.h"



/**
 * Dynamic data is usually in local coordinate system!.
 */
struct DynamicData {

    //Force in Newtons.
    Vector force;
    //Torqe in Nm.
    Vector torqe;
    //Data timestamp in microseconds
    uint32_t timestamp = 0;
    
};



#endif