#ifndef MPU9250_DRIVER_TEMPLATE_H
#define MPU9250_DRIVER_TEMPLATE_H



#include "definitions.h"

#include "interval_control.h"

#include "modules/module_template.h"

#include "mpu9250.h"
#include "CircularBuffer.h"


class MPU9250Driver: public Module {
public:

    
    /**
     * This is where all calculations are done.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns rate (in Hz) of the thread
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t loopRate() {return _loopRate;};

    /**
     * Returns true if gyro data available
     *
     * @param values none.
     * @return bool.
     */
    bool gyroAvailable() {return !_gyroFifo.isEmpty();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t gyroRate() {return _gyroRate;};

    /**
     * Returns true if gyro data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool getGyro(Vector* gyroData, uint32_t* gyroTimestamp) {

        if (_gyroFifo.isEmpty()) return false;

        *gyroData = _gyroFifo.pop();
        *gyroTimestamp = _gyroTimestampFifo.pop();

        return true;

    };

    /**
     * Returns true if gyro data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool peekGyro(Vector* gyroData, uint32_t* gyroTimestamp) {

        if (_gyroFifo.isEmpty()) return false;

        *gyroData = _gyroFifo.last();
        *gyroTimestamp = _gyroTimestampFifo.last();

        return true;

    }

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushGyro() {
        _gyroFifo.clear();
        _gyroTimestampFifo.clear();
    }

    /**
     * Returns true if accel data available
     *
     * @param values none.
     * @return bool.
     */
    bool accelAvailable() {return !_accelFifo.isEmpty();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t accelRate() {return _accelRate;};

    /**
     * Returns true if accel data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool getAccel(Vector* accelData, uint32_t* accelTimestamp) {

        if (_accelFifo.isEmpty()) return false;

        *accelData = _accelFifo.pop();
        *accelTimestamp = _accelTimestampFifo.pop();

        return true;

    };

    /**
     * Returns true if accel data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool peekAccel(Vector* accelData, uint32_t* accelTimestamp) {

        if (_accelFifo.isEmpty()) return false;

        *accelData = _accelFifo.last();
        *accelTimestamp = _accelTimestampFifo.last();

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushAccel() {
        _accelFifo.clear();
        _accelTimestampFifo.clear();
    }

    /**
     * Returns true if Magnetometer data available
     *
     * @param values none.
     * @return bool.
     */
    bool magAvailable() {return !_magFifo.isEmpty();};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t magRate() {return _magRate;};

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * This will remove sensor data from queue, peek will not.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool getMag(Vector* magData, uint32_t* magTimestamp) {

        if (_magFifo.isEmpty()) return false;

        *magData = _magFifo.pop();
        *magTimestamp = _magTimestampFifo.pop();

        return true;

    };

    /**
     * Returns true if Magnetometer data valid.
     * Variables given as parameters will be overridden.
     * Will not remove data from queue, get will.
     *
     * @param values Vector and uint32_t.
     * @return bool.
     */
    bool peekMag(Vector* magData, uint32_t* magTimestamp) {

        if (_magFifo.isEmpty()) return false;

        *magData = _magFifo.last();
        *magTimestamp = _magTimestampFifo.last();

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushMag() {
        _accelFifo.clear();
        _accelTimestampFifo.clear();
    }


private:

    static void _interruptRoutine();

    void _getData();


    CircularBuffer <Vector, 100> _gyroFifo;
    CircularBuffer <Vector, 100> _accelFifo;
    CircularBuffer <Vector, 100> _magFifo;
    CircularBuffer <uint32_t, 100> _gyroTimestampFifo;
    CircularBuffer <uint32_t, 100> _accelTimestampFifo;
    CircularBuffer <uint32_t, 100> _magTimestampFifo;

    Vector _lastGyro;
    Vector _lastAccel;
    Vector _lastMag;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    Mpu9250 _imu = Mpu9250(&SPI, MPU_NCS_PIN);

    uint8_t _startAttempts = 0;

    uint32_t _loopRate = 0;
    uint32_t _loopCounter = 0;

    uint32_t _gyroRate = 0;
    uint32_t _gyroCounter = 0;

    uint32_t _accelRate = 0;
    uint32_t _accelCounter = 0;

    uint32_t _magRate = 0;
    uint32_t _magCounter = 0;

    uint32_t _lastMeasurement = 0;

    bool _block = false;

    static uint32_t _newDataTimestamp;
    static bool _newDataInterrupt;



    
};


extern MPU9250Driver IMU;





#endif