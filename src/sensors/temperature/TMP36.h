#ifndef WARDUINO_SENSOR_TMP36_H
#define WARDUINO_SENSOR_TMP36_H

#include "../Sensor.h"

namespace wlp {

    class TMP36 : public Sensor {
        pin_number m_pin;

    public:
        explicit TMP36(pin_number pin);

        int16_t readRaw();

        float readValue() override;
    };

}

#endif //WARDUINO_SENSOR_TMP36_H
