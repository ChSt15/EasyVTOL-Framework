#ifndef STARSHIP_H
#define STARSHIP_H



#include "definitions.h"

#include "utils/interval_control.h"

#include "data_containers/navigation_data.h"

#include "vehicle/vehicle_template.h"

#include "navigation_modules/navigation_complementary.h"
#include "guidance_modules/guidance_flybywire.h"



#define VEHICLE_LOOP_RATE 8000



class Starship: public Vehicle {
public:

    Starship() {
        //_connectThread(); //Add thread to threadChain.
        _navigation = &_navigationComp; //Set default navigation module.
        _guidance = &_guidanceFBW;
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
     *
     * @param values none.
     * @return bool.
     */
    bool vehicleReady() {return _vehicleInitialized;}


private:

    //limits the vehicle loop rate
    IntervalControl _interval = IntervalControl(VEHICLE_LOOP_RATE);

    //Set to true when vehicle is ready for flight.
    bool _vehicleInitialized = false;

    //Default module to use at start
    NavigationComplementary _navigationComp;

    //Default module to use at start
    GuidanceFlyByWire _guidanceFBW;

    //static Navigation* _navigation;


};




#endif