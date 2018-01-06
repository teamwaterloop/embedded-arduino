/**
 *  @file Waterloop_LSM9DS1.h
 *  @brief Accelerometer for 9 axis IMU sensor (LSM9DS1)
 *
 *  This file contains the prototypes the LSM9DS1 class, implemented in SFE_LSM9DS1.cpp.
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

#ifndef __Waterloop_LSM9DS1_H__
#define __Waterloop_LSM9DS1_H__

#include "Arduino.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"

/**
 * @brief I2C address of Accelerometer and Gyroscope
 *
 * The I2C address of Accelerometer and Gyroscope used for data accessing via
 * I2C channel for sensor chip
 */
#define _xgAddress 0x6B

/**
 * @brief enum list for axis definition
 *
 * Define the Axis for each data
 */
enum lsm9ds1_axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ALL_AXIS
};

/**
 *  @brief LSM9DS1 sensor class
*/
class LSM9DS1
{
public:

    /** @brief Sensor Class Default Constructor
     *
     *  Preset the default IMU Sensor Settings and private variables
     *  via Init() function
     *
     */
    LSM9DS1();

    /** @brief Initialize the accelerometer
     *
     *  This will set up the scale and output rate of each sensor.
     *  The values set in the IMUSettings struct will take effect
     *  after calling this function.
     *
     */
    uint16_t begin();

    /** @brief Calibrate the accelerometer
     *
     *  This is a function that uses the FIFO to accumulate sample
     *  of accelerometer and gyro data, average them, scales them to
     *  gs and deg/s respectively, and then passes the biases to the
     *  main sketch for subtraction from all subsequent data. There
     *  are no gyro and accelerometer bias registers to store the
     *  data as there are in the ADXL345, a precursor to the LSM9DS0, or
     *  the MPU-9150, so we have to subtract the biases ourselves.
     *  This results in a more accurate measurement in general and can
     *  remove errors due to imprecise or varying initial placement.
     *  Calibration of sensor data in this manner is a good practice.
     *
     *  @param autoCalc boolean value,
     *          true => enable the calibration value during calibration
     *
     */
    void calibrate(bool autoCalc = true);


    /** @brief Polls the accelerometer status register to check
     *         if new data is available.
     *
     *  @return	1 - New data available
     *          0 - No new data available
     *
     */
    uint8_t accelAvailable();

    /** @brief Polls the temperature status register to check
     *         if new data is available.
     *
     *  @return	1 - New data available
     *          0 - No new data available
     *
     */
    uint8_t tempAvailable();


    /** @brief Read the accelerometer output registers.
     *
     *  This function will read all six accelerometer output registers.
     *  The readings are stored in the class' ax, ay, and az private variables.
     *  Read those _after_ calling readAccel() by getAccX(), getAccY(), or getAccZ().
     *
     */
    void readAccel();

    /** @brief Read a specific axis of the accelerometer
     *
     *  This function will read and output a specific Axis.
     *  NOTE!: Every call will always update before returning data,
     *         therefore, every calls are not syncronized
     *
     *  @param axis ,which can be any of X_AXIS, Y_AXIS, or Z_AXIS.
     *  @return Acceleration value #about that particular axis #at the function call
     *
     */
    int16_t readAccel(lsm9ds1_axis axis);

    /** @brief Read the temperature output register.
     *
     *  This function will read two temperature output registers.
     *  The combined readings are stored in the class' private variables, temperature
     *  Use getTemp to obtain the readings
     *
     */
    void readTemp();

    /** @brief Convert from RAW signed 16-bit value to gravity (g's).
     *
     *  This function reads in a signed 16-bit value and returns the scaled
     *  g's. This function relies on aScale and aRes being correct.
     *
     *  @param accel = A signed 16-bit raw reading from the accelerometer.
     *  @return Converted acceleration value in [g's]
     *
     */
    float calcAccel(int16_t accel);

    /** @brief Set the full-scale range of the accelerometer.
     *
     *  This function can be called to set the scale of the accelerometer to
     *  2, 4, 6, 8, or 16 g's.
     *
     *  @param aScl = The desired accelerometer scale. Must be one of five possible
     *                values from the accel_scale.
     *
     */
    void setAccelScale(uint8_t aScl);

    /** @brief Set the output data rate of the accelerometer
     *
     *  @param aRate = The desired output rate of the accel.
     *
     */
    void setAccelODR(uint8_t aRate);

    /** @brief Configure Accelerometer Interrupt Generator
     *
     *  @param generator = Interrupt axis/high-low events
     *      Any OR'd combination of
     *      ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
     *  @param andInterrupts = AND/OR combination of interrupt events
     *	    true: AND combination
     *	    false: OR combination
     */
    void configAccelInt(uint8_t generator, bool andInterrupts = false);

    /** @brief Configure the threshold of an accelereomter axis
     *
     *  @param threshold = Interrupt threshold. Possible values: 0-255.
     *                     Multiply by 128 to get the actual raw accel value.
     *  @param axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
     *  @param duration = Duration value must be above or below threshold
     *                    to trigger interrupt
     *  @param wait = Wait function on duration counter
     *              true: Wait for duration samples before exiting interrupt
     *              false: Wait function off
     *
     */
    void configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration = 0, bool wait = 0);

    /** @brief Get contents of accelerometer interrupt source register
     */
    uint8_t getAccelIntSrc();

    /** @brief Get status of inactivity interrupt
     */
    uint8_t getInactivity();

    /** @brief Data Accessor Functions
     *
     *  These functions will return 16 bit integer value after calling reading functions
     *
     */
    int16_t getAccX(){return ax;};
    int16_t getAccY(){return ay;};
    int16_t getAccZ(){return az;};
    int16_t getTemp(){return temperature;};

protected:

      //////////////////////
     // Global Variables //
    /////////////////////
    IMUSettings settings;   //Settings struct for IMU
    int16_t ax, ay, az;     // x, y, and z axis readings of the accelerometer
    int16_t temperature;    // Chip temperature
    float aBias[3];         // accelerometer Bias values
    int16_t aBiasRaw[3];    // accelerometer raw Bias values

    /** @brief current resolution for accelerometer
     *
     *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
     *  This value is calculated as (sensor scale) / (2^15).
     *
     */
    float aRes;

    /** @brief Decision boolean for accounting aBias
     *
     *  this boolean value keeps track of whether we're automatically subtracting off
     *  accelerometer and gyroscope bias calculated in calibrate() fucntion calls
     *
     */
    bool _autoCalc;

      ///////////////
     // Functions //
    ///////////////
    /** @brief Set up accelerometer settings to default.
     *
     *  Default setting will be:    - I2C interface
     *                              - default accelerometer I2C Address
     *
     */
    void init();

    /** @brief Set up the accelerometer to begin reading.
     *
     *  This function steps through all accelerometer related control registers.
     *  Upon exit these registers will be set as:
     *	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
     *	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
     *		all axes enabled.
     *  - CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
     *  - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
     *
     */
    void initAccel();

    /** @brief Read a byte from a register in the accel/mag sensor
     *
     *  @param subAddress = Register to be read from.
     *  @return An 8-bit value read from the requested register.
     *
     */
    uint8_t xgReadByte(uint8_t subAddress);

    /** @brief Read a series of bytes from registers in series
     *
     *  Reads a series of bytes starting at an address from the accelerometer
     *
     *  @param subAddress   = Register to be read from.
     *  @param * dest       = A pointer to an array of uint8_t's. Values read
     *                        will be stored in here on Return.
     *  @param count        = The number of bytes to be read.
     *  @return No value is returned, but the `dest` array will store upon exiting
     *
     */
    uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

    /** @brief Write a byte to a register in the accel/mag sensor
     *
     *  @param subAddress   = Register to be read from.
     *  @param date       = data to be written to the register
     *
     */
    void xgWriteByte(uint8_t subAddress, uint8_t data);

    /** @brief Calculate the resolution of the accelerometer
     *
     *  This function will set the value of the aRes variable. aScale must
     *  be set prior to calling this function.
     *
     */
    void calcaRes();

    /** @brief Sleep or wake the gyroscope
     *
     *  @param enable :True = sleep gyro. False = wake gyro.
     *  be set prior to calling this function.
     *
     */
    void sleepGyro(bool enable = true);

    ///////////////////
    // I2C Functions //
    ///////////////////
    /** @brief Initialize the I2C hardware.
     *
     *  This function will setup all I2C pins and related hardware.
     *
     */
    void initI2C();

    /** @brief Write a byte out of I2C to a register in the device
     *
     *  @param address      = The 7-bit I2C address of the slave device.
     *  @param subAddress   = The register to be written to.
     *  @param data         = Byte to be written to the register.
     *
     */
    void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

    /** @brief Read a single byte from a register over I2C
     *
     *  @param address      = The 7-bit I2C address of the slave device
     *  @param subAddress   = The register to be read from
     *  @return The byte read from the requested address
     *
     */
    uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);

    /** @brief Read a series of bytes, starting at a register via I2C
     *
     *  @param address      = The 7-bit I2C address of the slave device
     *  @param subAddress   = The register to begin reading
     *  @param * dest       = Pointer to an array where we'll store the readings.
     *  @param count        = Number of registers to be read.
     *  @return No value is returned by the function, but the registers read are
     *          all stored in the *dest array given.
     *
     */
    uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

       ///////////////////////
      // STH. from public, //
     // @todo #NOTE: Idk wt* these are, I will figure out late //
    ///////////////////////
    /** @brief Configure FIFO mode and Threshold
     *
     *  @param fifoMode = Set FIFO mode to off, FIFO (stop when full), continuous, bypass
     *         #NOTE: {FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT}
     *  @param fifoThs  = FIFO threshold level setting
     *         #NOTE: Any value from 0-0x1F is acceptable.
     *
     */
    void setFIFO(fifoMode_type fifoMode, uint8_t fifoThs);

    /** @brief Get number of samples
    */
    uint8_t getFIFOSamples();

    /** @brief  Enable or disable the FIFO
    *
    *   @param enable: true = enable, false = disable.
    *
    */
    void enableFIFO(bool enable = true);
};

#endif // SFE_LSM9DS1_H //
