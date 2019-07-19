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

    this->threshold = 420;

}

void QrdSensor::init() {
    // TODO: change to INPUT (analog)
    pinMode(pin, INPUT);
}

double QrdSensor::get_read() {
    // must be between 0.0 and 1.0
    //Serial.print(this->value);
    //Serial.print("       ");
    return fmax(fmin((this->value - this->on_white) / (this->on_tape - this->on_white), 1.0), 0.0);
}

// TODO: calibrate
bool QrdSensor::is_on() {
    return this->value >= this->threshold;
}

void QrdSensor::set_on_threshold(int thresh) {
    this->threshold = thresh;
}
