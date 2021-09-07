#ifndef DYNAMIC_DATA_H
#define DYNAMIC_DATA_H



#include "lib/Math-Helper/src/3d_math.h"
#include "data_container_base.h"



/**
 * Dynamic data is usually in local coordinate system!.
 */
class DynamicData: public DataContainerTimestamped_Base {
public:

    /**
     * Initialises all values to 0.
     */
    DynamicData() {
        force = 0;
        torqe = 0;
    }

    /**
     * 
     * @param forceN Force to initialise with.
     * @param torqueN Torque to initialise with.
     */
    DynamicData(const Vector<>& forceN, const Vector<>& torqueN = 0) {
        force = forceN;
        torqe = torqueN;
    }

    //Force in Newtons.
    Vector<> force;
    //Torqe in Nm.
    Vector<> torqe;

    ///Adds all components together.
    DynamicData operator+ (const DynamicData& data) {
        return DynamicData(data.force + force, data.torqe + torqe);
    }

    ///Compound operator for adding all components.
    DynamicData& operator+= (const DynamicData& data) {
        force += data.force;
        torqe += data.torqe;
        return *this;
    }

    ///Subtracts all componentsw.
    DynamicData operator- (const DynamicData& data) {
        return DynamicData(force - data.force, torqe - data.torqe);
    }

    ///Compound operator for subtracting all components.
    DynamicData& operator-= (const DynamicData& data) {
        force -= data.force;
        torqe -= data.torqe;
        return *this;
    }


    
};



#endif