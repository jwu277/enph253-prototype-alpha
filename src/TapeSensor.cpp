#include <Arduino.h>

#include "QrdSensor.hpp"
#include "TapeSensor.hpp"

// Constructor
TapeSensor::TapeSensor(PinName left_qrd_pin, PinName right_qrd_pin)
    : left_qrd(left_qrd_pin), right_qrd(right_qrd_pin) {

    this->x = 0.0; // default value of 0.0

};

// Read data
void TapeSensor::update() {

    this->left_qrd.update();
    this->right_qrd.update();

    // TODO: compute the x
    double foo = 0.0;

    this->x = foo;

}

double TapeSensor::get_x() {
    return this->x;
}
