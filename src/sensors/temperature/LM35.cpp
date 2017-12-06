#include "LM35.h"

using namespace wlp;

LM35::LM35(pin_number pin)
        : m_pin(pin) {}

int16_t LM35::readRaw() {
    return analogRead(m_pin);
}

float LM35::readValue() {
    int raw_value = readRaw();
    float temperature = (500.0f * raw_value) / MAX_ANALOG;
    return temperature;
}
