#ifndef MANUAL_PROFILE_H
#define MANUAL_PROFILE_H



#include "vehicle_control/control_profile_template.h"

#include "modules/guidance_modules/guidance_flybywire.h"

#include "comms/ibus_rx.h"



class ManualControlProfile: public ControlProfile {
public:

    ManualControlProfile() {
        loopRate_Hz = 100;
    }

    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();



private:

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    //Guidance module given to vehicle to be used.
    GuidanceFlyByWire _guidanceFBW;

    
};





#endif