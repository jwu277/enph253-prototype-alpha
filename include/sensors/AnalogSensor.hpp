#ifndef ANALOGSENSOR
#define ANALOGSENSOR

#include <Arduino.h>
#include "sensors\BaseSensor.hpp"

// AnalogSensor is a base class for single-pin analog sensors
class AnalogSensor: public BaseSensor {

    protected:

        // input pin
        PinName pin;

        // current analog value (0-1023 ?)
        int value;
    
    public:

        // update read
        void update();

        // obtain the read value
        int get_value();

};

#endif
