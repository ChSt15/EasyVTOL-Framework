#ifndef GUIDANCE_STATIONARY_H
#define GUIDANCE_STATIONARY_H



#include "guidance_template.h"

#include "data_containers/kinematic_data.h"

#include "CircularBuffer.h"



/**
 * This guidance module is designed to follow a set 
 * of points (inlcuding all kinematic data like attitude).
 * This can be used to also hover and stay stationary.
 */
class GuidancePath: public Guidance {
public:

    /**
     * Tell guidance to go to a certain point, speed, attitude etc.
     * Will travel from last point to new point in a line.
     * 
     * Control could ignore attitude commands.
     * 
     *
     * @param values kinematicData.
     * @return none.
     */
    void toPoint(const KinematicData &endPoint) {

    }




private:

    CircularBuffer<KinematicData, 100> _path;


};





#endif