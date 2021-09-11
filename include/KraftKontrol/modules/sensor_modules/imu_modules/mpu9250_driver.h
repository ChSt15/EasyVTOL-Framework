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

#include "KraftKontrol/platforms/arduino_platform/arduino_pin_interrupt_hal.h"




class MPU9250Driver: public Gyroscope_Interface, public Accelerometer_Interface, public Module_Abstract, public Task_Abstract {
public:

    MPU9250Driver(int interruptPin, int chipSelect, SPIClass* spiBus): Task_Abstract(40000, eTaskPriority_t::eTaskPriority_Realtime), _imu(spiBus, chipSelect), pinInterrupt_(interruptPin, _interruptRoutine, true, false) {
        
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
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t gyroRate() {return _gyroRate;};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t accelRate() {return _accelRate;};

    /**
     * Returns rate (in Hz) of the new sensor data
     *
     * @param values none.
     * @return uint32_t.
     */
    uint32_t magRate() {return _magRate;};


private:

    static void _interruptRoutine();

    void _getData();

    Vector<> _lastGyro;
    Vector<> _lastAccel;
    Vector<> _lastMag;

    IntervalControl _rateCalcInterval = IntervalControl(1); 

    PinInterrupt_HAL pinInterrupt_;

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

    int64_t _lastMeasurement = 0;

    bool _block = false;

    static int64_t _newDataTimestamp;
    static bool _newDataInterrupt;



    
};



#endif