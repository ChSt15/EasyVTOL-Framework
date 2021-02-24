#ifndef DYNAMIC_DATA_H
#define DYNAMIC_DATA_H


#include "definitions.h"

#include "vector_math.h"

/**
 * Dynamic data is usually in local coordinate system!.
 */
struct DynamicData {

    //Force in Newtons.
    Vector force;
    //Torqe in Nm.
    Vector torqe;
    
};



#endif