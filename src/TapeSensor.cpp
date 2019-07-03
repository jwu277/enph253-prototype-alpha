#include <Arduino.h>

#include "QrdSensor.hpp"
#include "TapeSensor.hpp"

// Current operation:
// x-axis: x = 0 at centered, left-to-right
// For x = 0, tape between sensors (not under any)
// For x = -+ 1, tape only beneath right/left, respectively
// For x = -+ FAR_OFF, tape not under any: use state machine

// How far off the tape follower would be if both sensors are off tape
// (could be computed as 1 + tape width / sensor width), units of sensor widths
#define FAR_OFF 5.0


// Constructor
TapeSensor::TapeSensor(PinName left_qrd_pin, PinName right_qrd_pin)
    : left_qrd(left_qrd_pin), right_qrd(right_qrd_pin) {

    this->x = 0.0; // default value of 0.0
    this->state = CENTRE; // start neutral

}

// Read data
void TapeSensor::update() {

    TapeSensor::update_qrds();
    TapeSensor::update_state();

}

void TapeSensor::update_qrds() {
    this->left_qrd.update();
    this->right_qrd.update();
}

void TapeSensor::update_state() {

    bool left_on = this->left_qrd.is_on();
    bool right_on = this->right_qrd.is_on();
    
    if (left_on && right_on) {

        // Currently also interpreting two sensors on tape as being on centre

        // On centre
        this->x = 0.0;

    }
    else if (left_on && !right_on) {

        // To the right
        this->x = 1.0;
        this->state = TapeSensor::RIGHT;

    }
    else if (right_on && !left_on) {

        // To the left
        this->x = -1.0;
        this->state = TapeSensor::LEFT;

    }
    // At  this point, both tape sensors are off the tape
    else if (this->state == TapeSensor::RIGHT) {

        // Far to the right
        this->x = FAR_OFF;

    }
    else {

        // Far to the left
        this->x = -FAR_OFF;

    }

}

// For PID
double* TapeSensor::get_x_ptr() {
    return &(this->x);
}
