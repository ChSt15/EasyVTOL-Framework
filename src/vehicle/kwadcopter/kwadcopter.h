#ifndef KWADCOPTER_H
#define KWADCOPTER_H



#include "utils/interval_control.h"

#include "data_containers/navigation_data.h"

#include "vehicle/vehicle_template.h"

#include "modules/navigation_modules/navigation_complementary.h"
#include "modules/guidance_modules/guidance_flybywire.h"
#include "modules/control_modules/powered_hover_controller.h"

#include "kwad_dynamics.h"



class Kwadcopter: public Vehicle {
public:

    /*Kwadcopter() {
        //Set module pointers to defaults.
        _guidance = &_guidanceFBW;
        _navigation = &_navigationComp; 
        _control = &_linearController;
        _dynamics = &_kwadDynamics;
    }*/

    /**
     * Thread function of the vehicle. 
     * All calculations the vehicle ever has to do for its 
     * control will be done here.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that setups the vehicle. If not called
     * then on the first Thread run this will automatically 
     * be called.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns true if the vehicle is ready for flight.
     * Currently only once navigation has reached AHRS mode.
     *
     * @param values none.
     * @return bool.
     */
    bool vehicleReady() {return _vehicleInitialized && (_navigation->getNavigationData().attitudeMode == NAV_ATTITUDE_MODE::AHRS);}


private:

    //Set to true when vehicle is ready for flight.
    bool _vehicleInitialized = false;

    //Default module to use at start
    GuidanceFlyByWire _guidanceFBW;

    //Default module to use at start
    NavigationComplementary _navigationComp;

    //Default module to use at start
    HoverController _linearController;

    //Default module to use at start
    KwadDynamics _kwadDynamics;


};




#endif