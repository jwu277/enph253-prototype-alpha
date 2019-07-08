#include <Arduino.h>
#include "QrdSensor.hpp"

#define THRESHOLD 512

// Constructor
QrdSensor::QrdSensor(PinName pin) {
    this->pin = pin;
    this->value = 0; // Default value of 0
}

void QrdSensor::init() {
    pinMode(pin, INPUT_PULLUP);
}

bool QrdSensor::is_on() {
    return this->value >= THRESHOLD;
}
