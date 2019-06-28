#include <Arduino.h>
#include "AnalogSensor.hpp"

// Reads the analog device + stores result in state
void AnalogSensor::update() {
    this->value = analogRead(this->pin);
};

// Return the read value
int AnalogSensor::get_value() {
    return this->value;
}
