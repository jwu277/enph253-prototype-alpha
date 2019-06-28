#include <Arduino.h>
#include "QrdSensor.hpp"

// Constructor
QrdSensor::QrdSensor(PinName pin) {
    this->pin = pin;
    this->value = 0; // Default value of 0
};
