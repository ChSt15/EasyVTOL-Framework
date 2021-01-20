#ifndef BOARD_V_1_0_H
#define BOARD_V_1_0_H

/**
 * 
 * This defines all pins for a board and can be used to
 * easily change between boards and testing.
 * Board can be changed in definitions.h file by changing
 * the BOARD_VERSION_HEADER definition.
 * 
*/

#define RGBLED_PIN                  4

#define MPU_INT_PIN                 2
#define MPU_NCS_PIN                 3

#define BME280_NCS_PIN              6

#define QMC5883L_INT_PIN            16

#define SX1280_NSS_PIN              9
#define SX1280_NRESET_PIN           5
#define SX1280_RFBUSY_PIN           15
#define SX1280_DIO1_PIN             14
#define SX1280_RXEN_PIN             10
#define SX1280_TXEN_PIN             17

#define INPUT_VOLTAGE_ADC_PIN       26
#define INPUT_VOLTAGE_ADC_FACTOR    1.0f


#endif