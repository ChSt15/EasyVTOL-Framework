#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H



/**
 * Enum containing all vehicle modes.
 */
enum eVehicleMode_t {
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
 * Struct containing all vehicle data.
 */
struct VehicleData {

    eVehicleMode_t vehicleMode = eVehicleMode_t::eVehicleMode_Disarm;

};




#endif