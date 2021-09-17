#ifndef DATA_CONTAINER_BASE_H
#define DATA_CONTAINER_BASE_H


#include "stdint.h"


/**
 * Cannot be instantiated.
 * Inheret for timestamp data.
 */
class DataContainerTimestamped_Base {
public:

    int64_t timestamp;

protected:

    DataContainerTimestamped_Base(){}

};



#endif