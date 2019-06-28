  /*****************************************************************
   * This is testing code for waterloop library for IMU sensor
  
  The LSM9DS1 is a versatile 9DOF sensor. But we only use the accelerometer.
  
  This Arduino sketch is a demo of the simple side of the
  WATERLOOP_LSM9DS1 library. It'll demo the following:
  * How to create a LSM9DS1 object
  * How to use the begin() function of the LSM9DS1 class.
  * How to read the raw accelerometer using the readAccel() function
  
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
    LSM9DS1 --------- Arduino
     SCL ---------- SCL (A5 on older 'Duinos')
     SDA ---------- SDA (A4 on older 'Duinos')
     VDD ------------- 3.3V (max 3.6V!!! so 3.3V rail on arduino)
     GND ------------- GND

  Development environment specifics:
    IDE: Arduino 1.6.3
    Hardware Platform: SparkFun Redboard
    LSM9DS1 Breakout Version: 1.0
  *****************************************************************/
  // The SFE_LSM9DS1 library requires both Wire and SPI be
  // included BEFORE including the 9DS1 library.
  #include <Wire.h>
  #include <SPI.h>
  #include <Waterloop_LSM9DS1.h>
  
  //////////////////////////
  // LSM9DS1 Library Init //
  //////////////////////////
  // Use the LSM9DS1 class to create an object. [imu] can be
  // named anything, we'll refer to that throught the sketch.
  LSM9DS1 imu;
  
  ////////////////////////////
  // Sketch Output Settings //
  ////////////////////////////
  //#define PRINT_RAW
  #define PRINT_SPEED 250 // 250 ms between prints
  static unsigned long lastPrint = 0; // Keep track of print time
  
  void setup() 
  {
    
    Serial.begin(115200);
    
    // The above lines will only take effect AFTER calling
    // imu.begin(), which verifies communication with the IMU
    // and turns it on.
    if (!imu.begin())
    {
      Serial.println("Failed to communicate with LSM9DS1.");
      Serial.println("Double-check wiring.");
      Serial.println("Default settings in this sketch will " \
                    "work for an out of the box LSM9DS1 " \
                    "Breakout, but may need to be modified " \
                    "if the board jumpers are.");
      while (1)
        ;
    }
  }
  
  void loop()
  {
     //Serial.println("Loop");
    if ( imu.accelAvailable() )
    {
      // To read from the accelerometer, first call the
      // readAccel() function. When it exits, it'll update the
      // ax, ay, and az variables with the most current data.
      imu.readAccel();
    }
    if ((lastPrint + PRINT_SPEED) < millis())
    {
      printAccel(); // Print "A: ax, ay, az"
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }
  
  
  void printAccel()
  {  
      // Now we can use the ax, ay, and az variables as we please.
      // Either print them as raw ADC values, or calculated in g's.
      Serial.print("A: ");
      Serial.print(imu.getAccX());
      Serial.print(", ");
      Serial.print(imu.getAccY());
      Serial.print(", ");
      Serial.println(imu.getAccZ());
  }
  
