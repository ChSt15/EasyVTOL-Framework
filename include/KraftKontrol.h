#ifndef KRAFT_KONTROL_H
#define KRAFT_KONTROL_H


#include "KraftKontrol/platforms/platform_hal.h"

#include "KraftKontrol/modules/communication_modules/kraft_kommunication.h"
#include "KraftKontrol/utils/Simple-Schedule/task_autorun_class.h"

#include "KraftKontrol/vehicle/vehicle_interface.h"
#include "KraftKontrol/vehicle/vehicle_general.h"

#include "KraftKontrol/modules/control_modules/control_interface.h"

#include "KraftKontrol/modules/datalink_modules/sx1280_lora_driver.h"

#include "KraftKontrol/modules/dynamics_modules/dynamics_interface.h"

#include "KraftKontrol/modules/guidance_modules/guidance_interface.h"
#include "KraftKontrol/modules/guidance_modules/guidance_flybywire.h"

#include "KraftKontrol/modules/system_models/system_model_abstract.h"
#include "KraftKontrol/modules/system_models/system_model_generic.h"

#include "KraftKontrol/modules/navigation_modules/navigation_abstract.h"
#include "KraftKontrol/modules/navigation_modules/navigation_complementary.h"

#include "KraftKontrol/modules/sensor_modules/gyroscope_modules/gyroscope_abstract.h"

#include "KraftKontrol/modules/sensor_modules/accelerometer_modules/accelerometer_abstract.h"

#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/magnetometer_abstract.h"

#include "KraftKontrol/modules/sensor_modules/imu_modules/mpu9250_driver.h"

#include "KraftKontrol/modules/sensor_modules/barometer_modules/barometer_abstract.h"
#include "KraftKontrol/modules/sensor_modules/barometer_modules/bme280_driver.h"

#include "KraftKontrol/modules/sensor_modules/gnss_modules/gnss_abstract.h"
#include "KraftKontrol/modules/sensor_modules/gnss_modules/ublox_serial_gnss.h"

#include "KraftKontrol/modules/sensor_modules/adc_modules/adc_abstract.h"
#include "KraftKontrol/modules/sensor_modules/adc_modules/ads1115_driver.h"

#include "KraftKontrol/modules/hid_modules/input_modules/button_abstract.h"
#include "KraftKontrol/modules/hid_modules/input_modules/gpio_button.h"
#include "KraftKontrol/modules/hid_modules/input_modules/analog_button.h"
#include "KraftKontrol/modules/hid_modules/input_modules/joystick.h"

#include "KraftKontrol/gui/gui.h"
#include "KraftKontrol/gui/menu_abstract.h"

#include "KraftKontrol/modules/hid_modules/display_modules/display_abstract.h"

#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_messagetype_abstracts.h"
#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_data_messages.h"
#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_telemetry_messages.h"
#include "KraftKontrol/KraftPacket_KontrolPackets/kraftkontrol_command_messages.h"

#include "KraftKontrol/utils/topic.h"
#include "KraftKontrol/utils/topic_subscribers.h"
#include "KraftKontrol/modules/communication_modules/kraft_message_subscriber.h"

#include "KraftKontrol/modules/networking_modules/kraft_konnect_network.h"



#endif 

