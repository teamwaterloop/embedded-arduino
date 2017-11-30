#include "TMP36.h"

using namespace wlp;

TMP36::TMP36(pin_number pin)
        : m_pin(pin) {}

int16_t TMP36::readRaw() {
    return analogRead(m_pin);
}

float TMP36::readValue() {
    int raw_value = readRaw();
    float voltage = raw_value * 5.0f / MAX_ANALOG;
    float temperature = (voltage - 0.5f) * 100.0f;
    return temperature;
}
