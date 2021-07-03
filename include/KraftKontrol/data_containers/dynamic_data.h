#ifndef DYNAMIC_DATA_H
#define DYNAMIC_DATA_H



#include "lib/Math-Helper/src/3d_math.h"
#include "data_container_base.h"



/**
 * Dynamic data is usually in local coordinate system!.
 */
class DynamicData: public DataContainerTimestamped_Base {
public:

    //Force in Newtons.
    Vector<> force;
    //Torqe in Nm.
    Vector<> torqe;
    
};



#endif