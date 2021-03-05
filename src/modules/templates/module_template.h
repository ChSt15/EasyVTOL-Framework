#ifndef MODULE_TEMPLATE_H
#define MODULE_TEMPLATE_H



#include "stdint.h"

#include "data_containers/vehicle_mode.h"

/**
 * This is a template that contains all the functions that all 
 * modules must have.
*/



class Module {
public:

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    virtual void thread() = 0;

    /**
     * Module specific init function that sets the 
     * module up.
     * 
     * A pointer to the vehicles mode must be given in order to
     * allow the module to always know if a failsafe of error has
     * occured.
     *
     * @param values vehicleMode.
     * @return none.
     */
    virtual void init(VEHICLE_MODE* vehicleMode) {
        _vehicleMode = vehicleMode;
        init();
    };


protected:

    virtual void init() = 0;

    static VEHICLE_MODE* _vehicleMode;


private:




};




#endif