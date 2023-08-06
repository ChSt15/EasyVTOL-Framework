#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H



/**
 * Enum containing all vehicle modes.
 */
enum eVehicleMode_t : uint8_t {
    //When this is set, the vehicle turns everything off due to a critical error in a subsystem. Should only be resetable from power reset.
    eVehicleMode_Error,
    //When this is set, the vehicle turns everything off. Should be resetable from vehicle.
    eVehicleMode_Failsafe,
    //Same as failsafe but indicates no error and puts vehicle into safe mode.
    eVehicleMode_Disarm,
    //Vehicle prepares for flight. Move actuators into position.
    eVehicleMode_Prepare,
    //When this is set, the vehicle turns everything on and follows flight commands.
    eVehicleMode_Arm
};


/**
 * Sets c_str to name for given mode.
 * @param c_str Char array to receive mode name.
 * @param c_strSize length of c_str.
 * @param mode Which mode to write into c_str
 */
inline void getVehicleModeString(char* c_str, uint32_t c_strSize, eVehicleMode_t mode) {

    switch (mode) {
    case eVehicleMode_t::eVehicleMode_Arm :
        strncpy(c_str, "Armed", c_strSize);
        break;

    case eVehicleMode_t::eVehicleMode_Disarm :
        strncpy(c_str, "Disarmed", c_strSize);
        break;

    case eVehicleMode_t::eVehicleMode_Error :
        strncpy(c_str, "Error", c_strSize);
        break;

    case eVehicleMode_t::eVehicleMode_Failsafe :
        strncpy(c_str, "Failsafe", c_strSize);
        break;

    case eVehicleMode_t::eVehicleMode_Prepare :
        strncpy(c_str, "Prepare", c_strSize);
        break;
    
    default:
        strncpy(c_str, "Unknown", c_strSize);
        break;
    }

}



/**
 * Struct containing all vehicle data.
 */
struct VehicleData {
    
    eVehicleMode_t vehicleMode = eVehicleMode_t::eVehicleMode_Disarm;

    //Is true when vehicle is ready to be armed
    bool vehicleReady = false;

};




#endif