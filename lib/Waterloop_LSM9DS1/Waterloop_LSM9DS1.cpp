/**
 *  @file Waterloop_LSM9DS1.cpp
 *  @brief Source File for Accelerometer for 9 axis IMU sensor (LSM9DS1)
 *
 *  This file implements all functions of the LSM9DS1 class. Functions here range
 *  from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
 *  hardware reads and writes. Both SPI and I2C handler functions can be found
 *  towards the bottom of this file.
 *
 *  @claim NOTE: This file is a modified file based on SparkFun open-source library.
 *  For our needs, it only contains the public accessors accelerometer and
 *  chip temperature readings. Other 6 axis such as magnetometer and gyroscope
 *  requires additional code.
 *  These code can be obtained from the Original Sensor Library:
 *  https://github.com/sparkfun/LSM9DS1_Breakout
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
 *  @todo Understand what is going on for FIFO calibration
 */


#include "Waterloop_LSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include <Wire.h> // Wire library is used for I2C
#include <SPI.h>  // SPI library is used for...SPI.
#include "Arduino.h"

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732

LSM9DS1::LSM9DS1()
{
    //Initialize private variables for sensor setttings including
    //scale, sampleRate, bandwidth, and resolution
	init();
}

void LSM9DS1::init()
{
	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3.
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = -1;
	settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;

	settings.temp.enabled = true;
	for (int i=0; i<3; i++)
	{
		aBias[i] = 0;
		aBiasRaw[i] = 0;
	}
	_autoCalc = false;
}


uint16_t LSM9DS1::begin()
{
	//constrainScales
    if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
        (settings.accel.scale != 8) && (settings.accel.scale != 16))
    {
        settings.accel.scale = 2;
    }

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	// Now, initialize our hardware interface.
	initI2C();	// Initialize I2C

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	//uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8); //| mTest;

    //!!!NOTE:Need to work on here??? for masking
	// if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
	// 	return 0;

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void LSM9DS1::initAccel()
{
	uint8_t tempRegValue = 0;

	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);

	xgWriteByte(CTRL_REG5_XL, tempRegValue);

	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
	}
	switch (settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}


void LSM9DS1::calibrate(bool autoCalc)
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};

	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
	}
	for(ii = 0; ii < samples ; ii++)
	{
		readAccel();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}
	for (ii = 0; ii < 3; ii++)
	{
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = calcAccel(aBiasRaw[ii]);
	}

	enableFIFO(false);
	setFIFO(FIFO_OFF, 0x00);

	if (autoCalc) _autoCalc = true;
}

uint8_t LSM9DS1::accelAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return (status & (1<<0));
}

uint8_t LSM9DS1::tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);

	return ((status & (1<<2)) >> 2);
}

void LSM9DS1::readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	if ( xgReadBytes(OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
	{
		ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
		ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
		az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
		if (_autoCalc)
		{
			ax -= aBiasRaw[X_AXIS];
			ay -= aBiasRaw[Y_AXIS];
			az -= aBiasRaw[Z_AXIS];
		}
	}
}

int16_t LSM9DS1::readAccel(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	if ( xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];

		if (_autoCalc)
			value -= aBiasRaw[axis];

		return value;
	}
	return 0;
}


void LSM9DS1::readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp
	if ( xgReadBytes(OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		temperature = ((int16_t)temp[1] << 8) | temp[0];
	}
}

float LSM9DS1::calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

void LSM9DS1::setAccelScale(uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;

	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);

	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS1::setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
}

void LSM9DS1::calcaRes()
{
	switch (settings.accel.scale)
	{
	case 2:
		aRes = SENSITIVITY_ACCELEROMETER_2;
		break;
	case 4:
		aRes = SENSITIVITY_ACCELEROMETER_4;
		break;
	case 8:
		aRes = SENSITIVITY_ACCELEROMETER_8;
		break;
	case 16:
		aRes = SENSITIVITY_ACCELEROMETER_16;
		break;
	default:
		break;
	}
}

// void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator,
// 	                     h_lactive activeLow, pp_od pushPull)
// {
// 	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
// 	// those two values.
// 	// [generator] should be an OR'd list of values from the interrupt_generators enum
// 	xgWriteByte(interrupt, generator);
//
// 	// Configure CTRL_REG8
// 	uint8_t temp;
// 	temp = xgReadByte(CTRL_REG8);
//
// 	if (activeLow) temp |= (1<<5);
// 	else temp &= ~(1<<5);
//
// 	if (pushPull) temp &= ~(1<<4);
// 	else temp |= (1<<4);
//
// 	xgWriteByte(CTRL_REG8, temp);
// }

uint8_t LSM9DS1::getInactivity()
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void LSM9DS1::configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void LSM9DS1::configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);

	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1::getAccelIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);

	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}

	return 0;
}


void LSM9DS1::sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1::getFIFOSamples()
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}

void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
	I2CwriteByte(_xgAddress, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
    // Whether we're using I2C, read multiple bytes using the
	// gyro-specific I2C address pin.
	return I2CreadByte(_xgAddress, subAddress);
}

uint8_t LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C, read multiple bytes using the
	// gyro-specific I2C address pin.
	return I2CreadBytes(_xgAddress, subAddress, dest, count);
}


void LSM9DS1::initI2C()
{
	Wire.begin();	// Initialize I2C library
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data

	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address

	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	byte retVal;
	Wire.beginTransmission(address);      // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);        // Put slave register address in Tx buffer
	retVal = Wire.endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
	if (retVal != 0) // endTransmission should return 0 on success
		return 0;

	retVal = Wire.requestFrom(address, count);  // Read bytes from slave register address
	if (retVal != count)
		return 0;

	for (int i=0; i<count;)
		dest[i++] = Wire.read();

	return count;
}
