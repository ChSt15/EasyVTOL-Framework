#ifndef PLATFORM_HAL_H
#define PLATFORM_HAL_H


/**
 * This header contains the directories to different HAL implementations according to platforms.
 * The Implementations should be automatically selected, depending on build environment.
 */

//Arduino platform implementations
#ifdef Arduino_h

#include "arduino_platform/arduino_gpio_hal.h"
#include "arduino_platform/arduino_i2c_busdevice_hal.h"
#include "arduino_platform/arduino_spi_busdevice_hal.h"
#include "arduino_platform/arduino_systemtime.h"
#include "arduino_platform/arduino_pin_interrupt_hal.h"

#endif


//RODOS platform implementations
#ifdef __RODOS_H__

#include "rodos_platform/rodos_i2c_hal.h"
#include "rodos_platform/rodos_spi_hal.h"
#include "rodos_platform/rodos_systemtime.h"
#include "rodos_platform/rodos_pin_interrupt_hal.h"

#endif



#endif 
