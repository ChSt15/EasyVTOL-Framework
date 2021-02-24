#ifndef STARSHIP_H
#define STARSHIP_H



#include "definitions.h"

#include "vehicle/vehicle_template.h"

#include "navigation_modules/navigation_complementary.h"

#include "utils/interval_control.h"


#define VEHICLE_LOOP_RATE 8000



class Starship: public Vehicle {
public:

    Starship() {
        //_connectThread(); //Add thread to threadChain.
        _navigation = &_navigationDefault; //Set default navigation module.
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

    //Module that takes care of navigation (sensorfusion etc.).
    NavigationComplementary _navigationDefault;

    //static Navigation* _navigation;


};




#endif