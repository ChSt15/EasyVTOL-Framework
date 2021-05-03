#ifndef MPU9250_DRIVER_TEMPLATE_H
#define MPU9250_DRIVER_TEMPLATE_H



#include "Arduino.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "modules/sensor_modules/gyroscope_modules/gyroscope_interface.h"
#include "modules/sensor_modules/accelerometer_modules/accelerometer_interface.h"
#include "modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"

#include "modules/module_abstract.h"

#include "lib/MPU9250_Lib/src/mpu9250.h"

#include "boards/board_v_1_0.h"

#include "utils/circular_buffer.h"



class MPU9250Driver: public Gyroscope_Interface, public Accelerometer_Interface, public Magnetometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    MPU9250Driver(int interruptPin, int chipSelect, SPIClass* spiBus) : _imu(spiBus, chipSelect), Task_Abstract(35000, eTaskPriority_t::eTaskPriority_Realtime, true) {
        imuINTPin_ = interruptPin;
    }
    
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
    bool gyroAvailable() {return _gyroFifo.available();};

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

        if (!_gyroFifo.available()) return false;

        *gyroData = _gyroFifo.pop_back();
        *gyroTimestamp = _gyroTimestampFifo.pop_back();

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

        if (!_gyroFifo.available()) return false;

        *gyroData = *_gyroFifo.peek_back();
        *gyroTimestamp = *_gyroTimestampFifo.peek_back();

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
    bool accelAvailable() {return _accelFifo.available();};

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

        if (!_accelFifo.available()) return false;

        *accelData = _accelFifo.pop_back();
        *accelTimestamp = _accelTimestampFifo.pop_back();

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

        if (!_accelFifo.available()) return false;

        *accelData = *_accelFifo.peek_back();
        *accelTimestamp = *_accelTimestampFifo.peek_back();

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
    bool magAvailable() {return _magFifo.available();};

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

        if (!_magFifo.available()) return false;

        *magData = _magFifo.pop_back();
        *magTimestamp = _magTimestampFifo.pop_back();

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

        if (!_magFifo.available()) return false;

        *magData = *_magFifo.peek_back();
        *magTimestamp = *_magTimestampFifo.peek_back();

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


    Circular_Buffer <Vector, 100> _gyroFifo;
    Circular_Buffer <Vector, 100> _accelFifo;
    Circular_Buffer <Vector, 100> _magFifo;
    Circular_Buffer <uint32_t, 100> _gyroTimestampFifo;
    Circular_Buffer <uint32_t, 100> _accelTimestampFifo;
    Circular_Buffer <uint32_t, 100> _magTimestampFifo;

    Vector _lastGyro;
    Vector _lastAccel;
    Vector _lastMag;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    int imuINTPin_ = 0;

    Mpu9250 _imu;

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



#endif