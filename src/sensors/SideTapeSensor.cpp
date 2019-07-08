#include <Arduino.h>

#include "sensors\QrdSensor.hpp"
#include "sensors\SideTapeSensor.hpp"

// Constructor
SideTapeSensor::SideTapeSensor(PinName left_side_qrd_pin, PinName right_side_qrd_pin)
    : left_side_qrd(left_side_qrd_pin), right_side_qrd(right_side_qrd_pin) {

    this->left_on = false; // default false
    this->right_on = false; // default false

}

void SideTapeSensor::init() {

    left_side_qrd.init();
    right_side_qrd.init();

}

// Read data
void SideTapeSensor::update() {

    SideTapeSensor::update_qrds();
    SideTapeSensor::update_state();

}

void SideTapeSensor::update_qrds() {
    this->left_side_qrd.update();
    this->right_side_qrd.update();
}

void SideTapeSensor::update_state() {

    this->left_on = this->left_side_qrd.is_on();
    this->right_on = this->right_side_qrd.is_on();

}

bool SideTapeSensor::is_left_on() {
    return this->left_on;
}

bool SideTapeSensor::is_right_on() {
    return this->right_on;
}
