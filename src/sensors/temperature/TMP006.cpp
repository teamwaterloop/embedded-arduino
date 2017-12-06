#include "TMP006.h"

using namespace wlp;

TMP006::TMP006(address i2c_addr, uint16_t sample_rate, Mode read_mode)
        : m_register(i2c_addr),
          m_sample_rate(sample_rate),
          m_mode(read_mode) {}

bool TMP006::begin() {
    m_register.begin();
    m_register.write16(TMP006_CONFIG, TMP006_CFG_MODEON | TMP006_CFG_DRDYEN | m_sample_rate);

    uint16_t man_id = m_register.read16(TMP006_MAN_ID);
    uint16_t dev_id = m_register.read16(TMP006_DEV_ID);
    return man_id == TMP006_EX_MAN_ID && dev_id == TMP006_EX_DEV_ID;
}

void TMP006::sleep() {
    uint16_t config = m_register.read16(TMP006_CONFIG);
    config &= ~(TMP006_CFG_MODEON);
    m_register.write16(TMP006_CONFIG, config);
}

void TMP006::wake() {
    uint16_t config = m_register.read16(TMP006_CONFIG);
    config |= TMP006_CFG_MODEON;
    m_register.write16(TMP006_CONFIG, config);
}

int16_t TMP006::readRawDieTemperature() {
    int16_t raw_value = m_register.read16(TMP006_TAMB);
    raw_value >>= 2;
    return raw_value;
}

int16_t TMP006::readRawVoltage() {
    return m_register.read16(TMP006_VOBJ);
}

double TMP006::readDieTemperature() {
    return readRawDieTemperature() * 0.03125;
}

double TMP006::readObjTemperature() {
    double die_temp = readDieTemperature() + 273.15;
    double vobj = readRawVoltage() * 1.5625e-7;
    double die_ref_temp = die_temp - TMP006_TREF;
    double die_ref_temp_sq = die_ref_temp * die_ref_temp;
    double S = 1 +
            TMP006_A1 * die_ref_temp +
            TMP006_A2 * die_ref_temp_sq;
    S *= TMP006_S0 * 1e-14;
    double vos = TMP006_B0 +
            TMP006_B1 * die_ref_temp +
            TMP006_B2 * die_ref_temp_sq;
    double vdiff = vobj - vos;
    double f_vobj = vdiff + TMP006_C2 * vdiff * vdiff;
    double die_temp_sq = die_temp * die_temp;
    double temperature = sqrt(sqrt(die_temp_sq * die_temp_sq + f_vobj / S));
    return temperature - 273.15;
}

float TMP006::readValue() {
    switch (m_mode) {
        case DIE:
            return static_cast<float>(readDieTemperature());
        case OBJECT:
            return static_cast<float>(readObjTemperature());
        default:
            return 0.0f;
    }
}
