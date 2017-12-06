#ifndef WARDUINO_SENSOR_LM35_H
#define WARDUINO_SENSOR_LM35_H

#include "../Sensor.h"

namespace wlp {

    class LM35 : public Sensor {
        pin_number m_pin;

    public:
        explicit LM35(pin_number pin);

        int16_t readRaw();

        float readValue() override;
    };

}

#endif //WARDUINO_SENSOR_LM35_H
