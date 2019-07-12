#include <Arduino.h>

#include "sensors/QrdSensor.hpp"

// On tape = 1.0, on white = 0.0

using namespace std;

// Constructor
QrdSensor::QrdSensor(PinName pin, tuple<int, int> thresholds) {

    this->pin = pin;

    this->on_white = get<0>(thresholds);
    this->on_tape = get<1>(thresholds);

    this->value = 0; // Default value of 0

}

void QrdSensor::init() {
    pinMode(pin, INPUT_PULLUP);
}

double QrdSensor::get_read() {
    // must be between 0.0 and 1.0
    //Serial.print(this->value);
    //Serial.print("       ");
    return fmax(fmin((this->value - this->on_white) / (this->on_tape - this->on_white), 1.0), 0.0);
}

// TODO: calibrate
bool QrdSensor::is_on() {
    return this->value >= (this->on_white + this->on_tape) / 2;
}
