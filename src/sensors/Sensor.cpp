#include "Sensor.h"

using namespace wlp;

float Sensor::readValue() {
    return 0.0f;
}

float Sensor::read() {
    float value = readValue();
    valueRead(value);
    return value;
}

void Sensor::valueRead(float value) {}
