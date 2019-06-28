#include <Arduino.h>
#include "AnalogSensor.hpp"

// Reads the analog device + stores result in state
int AnalogSensor::read() {
    this->value = analogRead(this->pin);
    return this->value;
};
