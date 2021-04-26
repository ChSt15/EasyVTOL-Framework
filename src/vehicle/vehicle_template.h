#ifndef VEHICLE_TEMPLATE_H
#define VEHICLE_TEMPLATE_H



#include "definitions.h"

#include "TaskScheduler.h"

#include "data_containers/kinematic_data.h"
#include "data_containers/vehicle_data.h"

#include "modules/guidance_modules/guidance_template.h"
#include "modules/navigation_modules/navigation_template.h"
#include "modules/control_modules/control_template.h"
#include "modules/dynamics_modules/dynamics_template.h"
#include "modules/module_template.h"



class Vehicle: public Module {
public:

    Vehicle(Guidance* guidancePointer, Navigation* navigationPointer, Control* controlPointer, Dynamics* dynamicsPointer) {
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


protected:

    //Inits vehicle scheduler
    void initScheduler(Guidance* guidancePointer, Navigation* navigationPointer, Control* controlPointer, Dynamics* dynamicsPointer);

    //Scheduler used by vehicle
    Scheduler _scheduler;

private:

    //Set to true when vehicle is ready for flight.
    bool _vehicleInitialized = false;

    //Points to the navigation module to use.
    static Navigation* _navigation;

    //Points to the guidance module to use.
    static Guidance* _guidance;

    //Points to the control module to use.
    static Control* _control;

    //Points to the dynamics module to use.
    static Dynamics* _dynamics;


};



#endif