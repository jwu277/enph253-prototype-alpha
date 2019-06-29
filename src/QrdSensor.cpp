#include <Arduino.h>
#include "QrdSensor.hpp"

#define THRESHOLD 512

// Constructor
QrdSensor::QrdSensor(PinName pin) {
    this->pin = pin;
    this->value = 0; // Default value of 0
};

bool QrdSensor::is_on() {
    return this->value >= THRESHOLD;
}
