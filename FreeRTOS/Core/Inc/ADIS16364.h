#ifndef ADIS16364_H
#define ADIS16364_H

#include "stm32f3xx_hal.h"

// Direcciones de los registros del ADIS16364
#define ADIS16364_XGYRO_OUT  0x04
#define ADIS16364_YGYRO_OUT  0x06
#define ADIS16364_ZGYRO_OUT  0x08
#define ADIS16364_XACCL_OUT  0x0A
#define ADIS16364_YACCL_OUT  0x0C
#define ADIS16364_ZACCL_OUT  0x0E

#define ADIS16364_XGYRO_OFF_LSB  0x1A
#define ADIS16364_XGYRO_OFF_MSB  0x1B
#define ADIS16364_YGYRO_OFF_LSB  0x1C
#define ADIS16364_YGYRO_OFF_MSB  0x1D
#define ADIS16364_ZGYRO_OFF_LSB  0x1E
#define ADIS16364_ZGYRO_OFF_MSB  0x1F
#define ADIS16364_XACCL_OFF_LSB  0x20
#define ADIS16364_XACCL_OFF_MSB  0x21
#define ADIS16364_YACCL_OFF_LSB  0x22
#define ADIS16364_YACCL_OFF_MSB  0x23
#define ADIS16364_ZACCL_OFF_LSB  0x24
#define ADIS16364_ZACCL_OFF_MSB  0x25

// #define ADIS16364_POWER_CTRL   0x3E
// Dato para comenzar una secuencia de burst read data
#define ADIS16364_BURST_UPPER_BYTE 0x3E
#define ADIS16364_BURST_LOWER_BYTE 0x00

// Bits de control en el registro POWER_CTRL
// #define ADIS16364_MODE_ON          0x4000
// #define ADIS16364_MODE_OFF         0x0000

#endif /* ADIS16364_H */
