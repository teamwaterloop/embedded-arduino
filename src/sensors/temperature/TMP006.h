#ifndef WARDUINO_SENSORS_TMP006_H
#define WARDUINO_SENSORS_TMP006_H

#include "../Sensor.h"
#include "../../register/I2CRegister.h"

#define TMP006_DEFAULT_ADDRESS 0x40

#define TMP006_B0 (-2.94e-5)
#define TMP006_B1 (-5.7e-7)
#define TMP006_B2 (4.63e-9)
#define TMP006_C2 (13.4)
#define TMP006_TREF (298.15)
#define TMP006_A2 (-1.678e-5)
#define TMP006_A1 (1.75e-3)
#define TMP006_S0 (6.4) // * 10^-14

#define TMP006_I2C_ADDR 0x40
#define TMP006_CONFIG     0x02
#define TMP006_MAN_ID 0xFE
#define TMP006_DEV_ID 0xFF
#define TMP006_VOBJ  0x0
#define TMP006_TAMB 0x01

#define TMP006_CFG_RESET    0x8000
#define TMP006_CFG_MODEON   0x7000
#define TMP006_CFG_1SAMPLE  0x0000
#define TMP006_CFG_2SAMPLE  0x0200
#define TMP006_CFG_4SAMPLE  0x0400
#define TMP006_CFG_8SAMPLE  0x0600
#define TMP006_CFG_16SAMPLE 0x0800
#define TMP006_CFG_DRDYEN   0x0100
#define TMP006_CFG_DRDY     0x0080

#define TMP006_EX_MAN_ID 0x5449
#define TMP006_EX_DEV_ID 0x67

namespace wlp {
	
    class TMP006 : public Sensor {
    public:
        typedef uint8_t Mode;

    private:
        I2CRegister m_register;
        uint8_t m_sample_rate;
        Mode m_mode;

        int16_t readRawDieTemperature();
        int16_t readRawVoltage();

    public:
        enum {
            DIE, OBJECT, NUM_MODES
        };

        explicit TMP006(address i2c_addr = TMP006_I2C_ADDR,
                        uint8_t sample_rate = TMP006_CFG_16SAMPLE,
                        Mode read_mode = DIE);

        bool begin();
        void sleep();
        void wake();

        double readDieTemperature();
        double readObjTemperature();

        float readValue() override;
    };

}

#endif //WARDUINO_SENSORS_TMP006_H
