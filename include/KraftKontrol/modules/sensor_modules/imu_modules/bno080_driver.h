#ifndef BNO080_DRIVER_TEMPLATE_H
#define BNO080_DRIVER_TEMPLATE_H



#include "Arduino.h"

#include "KraftKontrol/utils/Simple-Schedule/task_threading.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"
#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_abstract.h"
#include "KraftKontrol/modules/navigation_modules/navigation_abstract.h"

#include "KraftKontrol/utils/value_error.h"

#include "KraftKontrol/modules/module_abstract.h"

#include "lib/SparkFun_BNO080/src/SparkFun_BNO080_Arduino_Library.h"

#include "KraftKontrol/utils/buffer.h"

#include "KraftKontrol/platforms/arduino_platform/arduino_pin_interrupt_hal.h"

#include "KraftKontrol/utils/topic_subscribers.h"

#include "lib/MathHelperLibrary/vector_math.h"




class BNO080Driver: /*public Gyroscope_Interface, public Accelerometer_Interface, public Magnetometer_Interface*/ public Module_Abstract, public Navigation_Abstract, public Task_Threading {
public:

    BNO080Driver(int interruptPin, TwoWire& i2cBus, const Barometer_Abstract* baro = nullptr, const GNSS_Abstract* gnss = nullptr);
    
    void thread();

    void init();


private:

    void getIMUData();
    void getGNSSData();
    void getBaroData();
    void prediction();

    PinInterrupt_HAL pinInterrupt_;

    TwoWire& i2c_;

    BNO080 _imu;

    uint32_t startAttempts_ = 0;

    int64_t lastLoopTimestamp_ = 0;

    float _lastHeightValue = 0;

    Buffer<VectorOLD<>, 50> accelBuf_;
    Buffer<VectorOLD<>, 50> gyroBuf_;

    Buffer<float, 20> baroHeightBuf_;
    Buffer<float, 20> baroVelBuf_;

    Buffer_Subscriber<DataTimestamped<float>, 10> baroSubr_;
    Simple_Subscriber<DataTimestamped<GNSSData>> gnssSubr_;

    volatile static int64_t _newDataTimestamp;
    volatile static bool _newDataInterrupt;
    static Task_Threading* driverTask_;
    static void _interruptRoutine();



    
};



#endif