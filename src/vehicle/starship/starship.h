#ifndef STARSHIP_H
#define STARSHIP_H



#include "utils/interval_control.h"

#include "data_containers/navigation_data.h"

#include "vehicle/vehicle_template.h"

#include "modules/navigation_modules/navigation_complementary.h"
#include "modules/guidance_modules/guidance_flybywire.h"
#include "modules/control_modules/powered_hover_controller.h"

#include "starship_dynamics.h"



class Starship: public Vehicle {
public:

    Vehicle(Guidance* guidancePointer, Navigation* navigationPointer, Control* controlPointer, StarshipDynamics* dynamicsPointer) {
        _guidance = guidancePointer;
        _navigation = navigationPointer;
        _control = controlPointer;
        _dynamics = dynamicsPointer;
    }

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




};




#endif