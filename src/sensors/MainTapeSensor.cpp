#include <Arduino.h>

#include "sensors/QrdSensor.hpp"
#include "sensors/MainTapeSensor.hpp"

// Current operation:
// x-axis: x = 0 at centered, left-to-right
// For x = 0, tape between sensors (not under any)
// For x = -+ 1, tape only beneath right/left, respectively
// For x = -+ FAR_OFF, tape not under any: use state machine

// How far off the tape follower would be if both sensors are off tape
// (could be computed as 1 + tape width / sensor width), units of sensor widths
#define FAR_OFF 1.3


// Constructor
MainTapeSensor::MainTapeSensor(PinName left_qrd_pin, PinName right_qrd_pin)
    : left_qrd(left_qrd_pin), right_qrd(right_qrd_pin) {

    this->x = 0.0; // default value of 0.0
    this->state = CENTRE; // start neutral

}

void MainTapeSensor::init() {

    left_qrd.init();
    right_qrd.init();

}

// Read data
void MainTapeSensor::update() {

    MainTapeSensor::update_qrds();
    MainTapeSensor::update_state();

}

void MainTapeSensor::update_qrds() {
    this->left_qrd.update();
    this->right_qrd.update();
}

void MainTapeSensor::update_state() {

    bool left_on = this->left_qrd.is_on();
    bool right_on = this->right_qrd.is_on();

    //pwm_start(PA_0, 1000000, 10, 0, 0);
    
    if (left_on && right_on) {

        // Currently also interpreting two sensors on tape as being on centre

        // On centre
        this->x = 0.0;

    }
    else if (left_on && !right_on) {

        // To the right
        this->x = 1.0;
        this->state = MainTapeSensor::RIGHT;

    }
    else if (right_on && !left_on) {

        // To the left
        this->x = -1.0;
        this->state = MainTapeSensor::LEFT;

    }
    // At  this point, both tape sensors are off the tape
    else if (this->state == MainTapeSensor::RIGHT) {

        // Far to the right
        this->x = FAR_OFF;

    }
    else {

        // Far to the left
        this->x = -FAR_OFF;

        //pwm_start(PA_0, 1000000, 10, 10, 0);

    }

}

bool MainTapeSensor::is_both_on() {
    return this->left_qrd.is_on() && this->right_qrd.is_on();
}

// For PID
double* MainTapeSensor::get_x_ptr() {
    return &(this->x);
}

void MainTapeSensor::set_state(State state) {
    this->state = state;
}