#ifndef SERVO_DYNAMICS_H
#define SERVO_DYNAMICS_H



#include "definitions.h"

#include "outputs/servo_ppm.h"



class ServoDynamics {
public: 

    /**
     * Servo channel is the channel that the ServoDynamics class controls,
     * maxSpeed is the maximum speed in radians per second that the servo can move at and 
     * max accel is the maximum acceleration in radians per second squared the servo can output.
     * Tuning these settings can help perfomance and also reduce strain on parts of the
     * vehicle that should not have sudden powerful movements. 
     * 
     * Position offset is added to the current output position (in radians) to correct for a servo offset.
     * Position scaler will then take the output (from 0 to 360) and scale it to the correct value (0 to 1)
     * So that the servo position will match that of the inputed position.
     *
     * @param values servoChannel, maxSpeed_radPerSec and maxAccel_radPerSecPerSec
     * @return none.
     */
    ServoDynamics(PPMChannel* servoChannel, const float &positionOffset, const float &maxSpeed_radPerSec, const float &maxAccel_radPerSecPerSec) {
        _channel = servoChannel;
        _maxSpeed = maxSpeed_radPerSec;
        _maxAccel = maxAccel_radPerSecPerSec;
        _positionOffset = positionOffset;
    }

    /**
     * Servo channel is the channel that the ServoDynamics class controls,
     * maxSpeed is the maximum speed in radians per second that the servo can move at and 
     * max accel is the maximum acceleration in radians per second squared the servo can output.
     * Tuning these settings can help perfomance and also reduce strain on parts of the
     * vehicle that should not have sudden powerful movements. 
     * 
     * Position offset is added to the current output position (in radians) to correct for a servo offset.
     * Position scaler will then take the output (from 0 to 360) and scale it to the correct value (0 to 1)
     * So that the servo position will match that of the inputed position.
     * 
     * @param values servoChannel, maxSpeed_radPerSec and maxAccel_radPerSecPerSec
     * @return none.
     */
    ServoDynamics(PPMChannel* servoChannel, const float &positionOffset, const float &maxSpeed_radPerSec, const float &maxAccel_radPerSecPerSec, float maxPosition, float minPosition) {
        _channel = servoChannel;
        _maxSpeed = maxSpeed_radPerSec;
        _maxAccel = maxAccel_radPerSecPerSec;
        _positionOffset = positionOffset;
        _maxPosition = maxPosition;
        _minPosition = minPosition;
    }

    /**
     * Lets the module calculate the new servoPosition.
     *
     * @param values none
     * @return none.
     */
    void thread() {

        float dTime = float(micros() - _lastThreadRunTimestamp_us)/1000000.0f;
        _lastThreadRunTimestamp_us = micros();


        float error = _setPosition - _currentPosition;

        
        float speedLimit = sqrtf(2*abs(error)*_maxAccel);

        if (error < 0) {
            speedLimit = -speedLimit;
        } 

        float acceleration = (speedLimit - _currentSpeed)/dTime;
        acceleration = constrain(acceleration, -_maxAccel, _maxAccel);

        _currentSpeed += acceleration*dTime;
        _currentSpeed = constrain(_currentSpeed, -_maxSpeed, _maxSpeed);

        _currentPosition += _currentSpeed*dTime;
        _currentPosition = constrain(_currentPosition, _minPosition, _maxPosition);

        _channel->setAngle(_currentPosition + _positionOffset, false);

    }

    /**
     * Sets the new wanted position in radians.
     *
     * @param values position in radians
     * @return none.
     */
    void setPosition(const float &position) {
        _setPosition = position;
    }

    /**
     * Returns the current servo position in radians
     *
     * @param values none
     * @return position in radians.
     */
    float getPosition() {
        return _currentPosition;
    }

    /**
     * Returns the current servo velocity in radians per second
     *
     * @param values none
     * @return position in radians.
     */
    float getVelocity() {
        return _currentSpeed;
    }


private:

    
    PPMChannel* _channel;

    float _maxSpeed;
    float _maxAccel;

    float _maxPosition = 3.1416;
    float _minPosition = -3.1416;

    float _positionOffset = 0.0f;

    float _currentPosition = 0;
    float _currentSpeed = 0;
    float _setPosition = 0;

    uint32_t _lastThreadRunTimestamp_us = 0;

    
};




#endif