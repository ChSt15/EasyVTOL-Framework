#include <Arduino.h>


Control controlModule;
Dynamics dynamicsModule;
Navigation navmodule(&GPSdriver, &IMUDriver, &MagDriver, &BaroDriver);
Guidance guidanceModule;

Radio radioModule; //This modules takes care of communication with the goundstation and radiocontrol. It receives waypoints or also manual control commands from controllers.

Vehicle vehicle(&guidanceModule, &navmodule, &controlModule, &dynamicsModule);


void vehicleProgram(); //Has all functions needed for vehicle testing, waypoints and stuff
void idleLoop(); //Will be ran whenever free time is available. DO NOT BLOCK e.g with delay(). Things that NEED to be ran should be placed in normal loop().
Runner runner(&vehicle, &vehicleProgram, &radioModule, &idleLoop)


void setup() {

    Serial.begin(115200);

}

void loop() {

    runner.tick();

}


void idleLoop() {

    //Do unimportant stuff. Will not run if whole system is too busy.

}


void vehicleProgram() {

    runner.armVehicle(); //Arms vehicle. Must be called before giving commands. This readies the vehicle.

    runner.vehicleBusy(); //Returns false if vehicle is ready for next command. Calling a command when vehicle isnt ready will make vehicle ignore old command and start with new command

    runner.p2pRelative(Vector(2,2,1), VEHICLE_WAIT, MAX_SPEED, MAX_ACCEL) //Reach at max this speed and accel. Cannot be higher than vehicles built in limits. Relative to current SETPOINT
    runner.p2pRelative(Vector(2,2,1), VEHICLE_WAIT, DTIME) //Reach at earliest this time
    runner.p2pRelative(Vector(2,2,1), VEHICLE_CONTINUE, DTIME) //VEHICLE_CONTINUE parameter will make vehicle follow this command but this will immediately return giving user power to do what they want in the mean time. Use runner.vehicleBusy() to find out if vehicle has finished command. runner.tick() MUST be called as fast as possible.

    runner.p2pAbsolute(Vector(2,2,1), VEHICLE_WAIT, same as relative) //simply go to point. Extra settings or like relative

    runner.p2pLinearRelative() //Moves in a straight line. Settings like non linear
    runner.p2pLinearRelative(Vector(Start), Vector(end)) //Same as p2pLinearRelative(Vector(end)) but will use the parameter start as starting line.

    runner.circleRelative(Vector(2,2,1), radius, forTime, ) //radius is radius of circle. forTime is the lenght of time. Use VEHICLE_CONTINUE for indefinate circle time and call runner.tick() as fast as possible.
    runner.circleRelative(Vector(2,2,1), Vector(startpoint), forradians) //startpoint is the point at which the circle starts. forradians is the angle the circle will go for. Use VEHICLE_CONTINUE for indefinate circle time and call runner.tick() as fast as possible.

    vehicle.currentState() //returns current kinematic state (position, velocity, accel, attitude, angularRate, angularAccel)
    vehicle.currentStateSetpoint() //returns current kinematic state setpoint. Same as currentState() but returns the setpoint. Usefull for some commands.

    //Some examples for lower level control:

    guidance.setSetpoints(kinematics) //vehicle modules can also be used for stuff like manual flight.

    float manualSettings[4];
    radio.getManualControl(manualSettings);
    dynamics = manualSettings //use manual settings for manual control and produce some dynamic mapping for full manual control.
    control.setDynamics(dynamics); //give vehicle controlModules manual dynamics and then call runner.tick() to have full manual control over vehicle. Same can be done with control module to have control module help reach kinematic setpoints like angular rate etc.



    runner.progEnd() //Will wait and keep running vehicle indefinetely. Will disarm vehicle and stop running this vehicleprogram loop.

}