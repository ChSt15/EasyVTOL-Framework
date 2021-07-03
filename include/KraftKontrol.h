#ifndef KRAFT_KONTROL_H
#define KRAFT_KONTROL_H



#include "lib/KraftKommunikation/src/kraft_kommunication.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/vehicle/vehicle_interface.h"
#include "KraftKontrol/vehicle/vehicle_general.h"

#include "KraftKontrol/modules/control_modules/control_interface.h"
#include "KraftKontrol/modules/control_modules/powered_hover_controller.h"

#include "KraftKontrol/modules/datalink_modules/sx1280_lora_driver.h"

#include "KraftKontrol/modules/dynamics_modules/dynamics_interface.h"

#include "KraftKontrol/modules/guidance_modules/guidance_interface.h"
#include "KraftKontrol/modules/guidance_modules/guidance_flybywire.h"
#include "KraftKontrol/modules/guidance_modules/guidance_path.h"

#include "KraftKontrol/modules/navigation_modules/navigation_interface.h"
#include "KraftKontrol/modules/navigation_modules/navigation_complementary.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_interface.h"

#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_interface.h"

#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_interface.h"

#include "KraftKontrol/modules/sensor_modules/imu_modules/mpu9250_driver.h"

#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_interface.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/bme280_driver.h"

#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_interface.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/ublox_serial_gnss.h"

#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_interface.h"
#include "KraftKontrol/modules/sensor_modules/adc_modules/ads1115_driver.h"

#include "KraftKontrol/modules/hid_modules/display_modules/display_interface.h"
#ifdef ESP32
    #include "KraftKontrol/modules/hid_modules/display_modules/st7735_driver.h"
#endif

#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_message_types.h"

#include "KraftKontrol/modules/networking_modules/kraft_konnect_network.h"



#endif 
