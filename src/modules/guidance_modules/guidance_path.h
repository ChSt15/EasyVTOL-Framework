#ifndef GUIDANCE_STATIONARY_H
#define GUIDANCE_STATIONARY_H



#include "task_autorun_class.h"

#include "guidance_interface.h"

#include "modules/module_abstract.h"

#include "data_containers/kinematic_data.h"

#include "utils/circular_buffer.h"

#include "interval_control.h"



/**
 * This guidance module is designed to follow a set 
 * of points (inlcuding all kinematic data like attitude).
 * This can be used to also hover and stay stationary.
 */
class GuidancePath: public Guidance_Interface, Task_Abstract {
public:

    /**
     * Creates a module based class and automatically adds it to the scheduler.
     * 
     * @param rate is the rate at which it will be ran at.
     * @param priority is the priority the module will have.
     */
    GuidancePath() : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_High, true) {}

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
        return false;
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


private:

    Circular_Buffer<KinematicData, 100> _path;

    ControlData vehicleControlSettings_;


};





#endif