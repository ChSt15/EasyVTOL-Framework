#ifndef MPU9250_DRIVER_TEMPLATE_H
#define MPU9250_DRIVER_TEMPLATE_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_interface.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_interface.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/MPU9250_Lib/src/mpu9250.h"

#include "KraftKontrol/utils/buffer.h"



class MPU9250Driver: public Gyroscope_Interface, public Accelerometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    MPU9250Driver(int interruptPin, int chipSelect, SPIClass* spiBus) : Task_Abstract(40000, eTaskPriority_t::eTaskPriority_Realtime, true), _imu(spiBus, chipSelect) {
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
    uint32_t gyroAvailable() {return _gyroFifo.available();};

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
    bool getGyro(Vector<>* gyroData, uint32_t* gyroTimestamp) {

        if (_gyroFifo.available() == 0) return false;

        _gyroFifo.takeBack(gyroData);
        _gyroTimestampFifo.takeBack(gyroTimestamp);

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
    bool peekGyro(Vector<>* gyroData, uint32_t* gyroTimestamp) {

        if (_gyroFifo.available() == 0) return false;

        _gyroFifo.peekBack(gyroData);
        _gyroTimestampFifo.peekBack(gyroTimestamp);

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
    uint32_t accelAvailable() {return _accelFifo.available();};

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
    bool getAccel(Vector<>* accelData, uint32_t* accelTimestamp) {

        if (_accelFifo.available() == 0) return false;

        _accelFifo.takeBack(accelData);
        _accelTimestampFifo.takeBack(accelTimestamp);

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
    bool peekAccel(Vector<>* accelData, uint32_t* accelTimestamp) {

        if (_accelFifo.available() == 0) return false;

        _accelFifo.peekBack(accelData);
        _accelTimestampFifo.peekBack(accelTimestamp);

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
    uint32_t magAvailable() {return _magFifo.available();};

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
    bool getMag(Vector<>* magData, uint32_t* magTimestamp) {

        if (_magFifo.available() == 0) return false;

        _magFifo.takeBack(magData);
        _magTimestampFifo.takeBack(magTimestamp);

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
    bool peekMag(Vector<>* magData, uint32_t* magTimestamp) {

        if (_magFifo.available() == 0) return false;

        _magFifo.peekBack(magData);
        _magTimestampFifo.peekBack(magTimestamp);

        return true;

    };

    /**
     * Removes all elements from queue.
     *
     * @param values none.
     * @return none.
     */
    void flushMag() {
        _magFifo.clear();
        _magTimestampFifo.clear();
    }


private:

    static void _interruptRoutine();

    void _getData();


    Buffer <Vector<>, 100> _gyroFifo;
    Buffer <Vector<>, 100> _accelFifo;
    Buffer <Vector<>, 100> _magFifo;
    Buffer <uint32_t, 100> _gyroTimestampFifo;
    Buffer <uint32_t, 100> _accelTimestampFifo;
    Buffer <uint32_t, 100> _magTimestampFifo;

    Vector<> _lastGyro;
    Vector<> _lastAccel;
    Vector<> _lastMag;

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