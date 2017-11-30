#ifndef WARDUINO_SENSOR_H
#define WARDUINO_SENSOR_H

#include <Arduino.h>

#define MIN_ANALOG 0.0f
#define MAX_ANALOG 1023.0f

namespace wlp {

    typedef uint8_t pin_number;

	class Sensor {
    public:
        virtual float readValue();

        float read();

        void valueRead(float value);
	};

}

#endif //WARDUINO_SENSOR_H
