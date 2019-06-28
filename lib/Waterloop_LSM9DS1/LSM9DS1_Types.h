/**
 *  @file LSM9DS1_Types.h
 *  @brief Types and Enumerations for Accelerometer for 9 axis IMU sensor (LSM9DS1)
 *
 *  This file defines all types and enumerations used by the LSM9DS1 class.
 *
 *  @claim NOTE: This file is a modified file based on SparkFun open-source library.
 *  It is modified in our needs, therefore, it would only work with modified source files.
 *
 *  @hardware LSM9DS1 Breakout Version: 1.0
 *
 *  @OriginalAuthor Jim Lindblom @ SparkFun Electronics
 *  @Original Creation Date: February 27, 2015
 *  @SourceReference https://github.com/sparkfun/LSM9DS1_Breakout
 *
 *  @author Jack Xu
 *  @First_Mod_Date December 5th, 2017
 *  @Last_Mod_Date January 5th, 2018
 *  @bug No known bugs.
 *  @QA Further QA required
 */


#ifndef __LSM9DS1_Types_H__
#define __LSM9DS1_Types_H__

#include "LSM9DS1_Registers.h"

// accel_scale defines all possible FSR's of the accelerometer:
enum accel_scale
{
	A_SCALE_2G,	// 00:  2g
	A_SCALE_16G,// 01:  16g
	A_SCALE_4G,	// 10:  4g
	A_SCALE_8G	// 11:  8g
};

// accel_oder defines all possible output data rates of the accelerometer:
enum accel_odr
{
	XL_POWER_DOWN, 	// Power-down mode (0x0)
	XL_ODR_10,		// 10 Hz (0x1)
	XL_ODR_50,		// 50 Hz (0x02)
	XL_ODR_119,		// 119 Hz (0x3)
	XL_ODR_238,		// 238 Hz (0x4)
	XL_ODR_476,		// 476 Hz (0x5)
	XL_ODR_952		// 952 Hz (0x6)
};

// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
enum accel_abw
{
	A_ABW_408,		// 408 Hz (0x0)
	A_ABW_211,		// 211 Hz (0x1)
	A_ABW_105,		// 105 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
};

enum interrupt_select
{
	XG_INT1 = INT1_CTRL,
	XG_INT2 = INT2_CTRL
};

enum interrupt_generators
{
	INT_DRDY_XL = (1<<0),	 // Accelerometer data ready (INT1 & INT2)
	INT_DRDY_G = (1<<1),	 // Gyroscope data ready (INT1 & INT2)
	INT1_BOOT = (1<<2),	 // Boot status (INT1)
	INT2_DRDY_TEMP = (1<<2),// Temp data ready (INT2)
	INT_FTH = (1<<3),		 // FIFO threshold interrupt (INT1 & INT2)
	INT_OVR = (1<<4),		 // Overrun interrupt (INT1 & INT2)
	INT_FSS5 = (1<<5),		 // FSS5 interrupt (INT1 & INT2)
	INT_IG_XL = (1<<6),	 // Accel interrupt generator (INT1)
	INT1_IG_G = (1<<7),	 // Gyro interrupt enable (INT1)
	INT2_INACT = (1<<7),	 // Inactivity interrupt output (INT2)
};

enum accel_interrupt_generator
{
	XLIE_XL = (1<<0),
	XHIE_XL = (1<<1),
	YLIE_XL = (1<<2),
	YHIE_XL = (1<<3),
	ZLIE_XL = (1<<4),
	ZHIE_XL = (1<<5),
	GEN_6D = (1<<6)
};

enum h_lactive
{
	INT_ACTIVE_HIGH,
	INT_ACTIVE_LOW
};

enum pp_od
{
	INT_PUSH_PULL,
	INT_OPEN_DRAIN
};

enum fifoMode_type
{
	FIFO_OFF = 0,
	FIFO_THS = 1,
	FIFO_CONT_TRIGGER = 3,
	FIFO_OFF_TRIGGER = 4,
	FIFO_CONT = 5
};

struct accelSettings
{
	// Accelerometer settings:
    uint8_t enabled;
    uint8_t scale;
    uint8_t sampleRate;
	// New accel stuff:
	uint8_t enableX;
	uint8_t enableY;
	uint8_t enableZ;
	int8_t  bandwidth;
	uint8_t highResEnable;
	uint8_t highResBandwidth;
};

struct temperatureSettings
{
	// Temperature settings
    uint8_t enabled;
};

struct IMUSettings
{
	accelSettings accel;
	temperatureSettings temp;
};

#endif
