#ifndef WARDUINO_REGISTER_H
#define WARDUINO_REGISTER_H

#include <Arduino.h>
#include <Wire.h>

namespace wlp {

    typedef uint16_t size_type;
    typedef uint8_t address;

    class I2CRegister {
        address m_i2c_addr;

    public:
        explicit I2CRegister(address i2c_addr);

        void begin();

        uint8_t read8(address reg_addr);
        uint16_t read16(address reg_addr);
        uint32_t read32(address reg_addr);
        uint64_t read64(address reg_addr);

        void write8(address reg_addr, uint8_t data);
        void write16(address reg_addr, uint16_t data);
        void write32(address reg_addr, uint32_t data);
        void write64(address reg_addr, uint64_t data);

        template<typename Int>
        Int readBytes(address reg_addr);

        template<typename Int>
        void writeBytes(address reg_addr, Int data);
    };

    template<typename Int>
    Int I2CRegister::readBytes(address reg_addr) {
        constexpr auto bytes = static_cast<size_type>(sizeof(Int));
        auto ret = static_cast<Int>(0);
        // Send register address from which to read
        Wire.beginTransmission(m_i2c_addr);
        Wire.write(reg_addr);
        Wire.endTransmission();
        // Read data from register
        Wire.beginTransmission(m_i2c_addr);
        for (size_type i = 0; i < bytes; ++i) {
            // Have to cast to ensure larger Int types are properly read
            ret = static_cast<Int>(static_cast<Int>(ret << 8) | static_cast<Int>(Wire.read()));
        }
        Wire.endTransmission(m_i2c_addr);
        return ret;
    }

    template<typename Int>
    void I2CRegister::writeBytes(address reg_addr, Int data) {
        constexpr auto bytes = static_cast<size_type>(sizeof(Int));
        // Send register address to which to write
        Wire.beginTransmission(m_i2c_addr);
        Wire.write(reg_addr);
        // Write byte data
        for (size_type i = 1; i <= bytes; ++i) {
            Wire.write(static_cast<uint8_t>(data >> (8 * (bytes - i))));
        }
        Wire.endTransmission();
    }

}

#endif //WARDUINO_REGISTER_H
