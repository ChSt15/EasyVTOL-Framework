#ifndef GUIDANCE_STATIONARY_H
#define GUIDANCE_STATIONARY_H



#include "guidance_template.h"

#include "data_containers/kinematic_data.h"

#include "CircularBuffer.h"

#include "utils/interval_control.h"



/**
 * This guidance module is designed to follow a set 
 * of points (inlcuding all kinematic data like attitude).
 * This can be used to also hover and stay stationary.
 */
class GuidancePath: public Guidance {
public:

    /**
     * Tells guidance to go to a certain point, speed, attitude etc.
     * Will travel from last point to new point in a line.
     * 
     * Control could ignore attitude commands.
     * 
     * Returns false if point buffer is full. Then the system has to first 
     * reach the next point before the next can be placed.
     * 
     *
     * @param values kinematicData.
     * @return bool.
     */
    bool toPoint(const KinematicData &endPoint) {

    }

    /**
     * Returns distance from endpoint. Used for switching between guidance modules.
     *
     * @param values none.
     * @return float.
     */
    float distanceFromEndpoint() {return 0.0;};

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();


private:

    CircularBuffer<KinematicData, 100> _path;

    IntervalControl _interval = IntervalControl(1000);


};





#endif