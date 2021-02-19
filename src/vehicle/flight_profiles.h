#ifndef FLIGHT_PROFILES_H
#define FLIGHT_PROFILES_H

/**
 * Enum containing all flight profiles.
 * These tell the vehicle how it should fly.
 */
enum FLIGHT_PROFILE {
    //Vehicle uses motors to produce lift.
    HOVER,
    //Vehicle uses surfaces to produce lift and moves continuesly
    FORWARD_FLIGHT,
    //No more powered flight. 
    GLIDE,
    //Alternate mode 1. Used if a vehicle has special flight profile.
    ALTERNATE_1,
    //Alternate mode 2. Used if a vehicle has special flight profile.
    ALTERNATE_2,
    //Alternate mode 3. Used if a vehicle has special flight profile.
    ALTERNATE_3,
    //Alternate mode 4. Used if a vehicle has special flight profile.
    ALTERNATE_4,
    //Alternate mode 5. Used if a vehicle has special flight profile.
    ALTERNATE_5,
    //Alternate mode 6. Used if a vehicle has special flight profile.
    ALTERNATE_6,
    //Alternate mode 7. Used if a vehicle has special flight profile.
    ALTERNATE_7,
    //Alternate mode 8. Used if a vehicle has special flight profile.
    ALTERNATE_8,
    //Alternate mode 9. Used if a vehicle has special flight profile.
    ALTERNATE_9,
    //Alternate mode 10. Used if a vehicle has special flight profile.
    ALTERNATE_10,

};



#endif
