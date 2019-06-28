#include <Arduino.h>
#include "QrdSensor.hpp"

// Reads the analog device + stores result in state
QrdSensor::QrdSensor(PinName pin) {
    this->pin = pin;
    this->value = 0; // Default value of 0
};
