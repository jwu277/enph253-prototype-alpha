#include <Arduino.h>
#include "DriveMotor.hpp"

//class constructor
DriveMotor::DriveMotor(PinName forward_pin, PinName reverse_pin, int pwm_clk_freq, int pwm_period)
    : PwmMotor::PwmMotor(pwm_clk_freq, pwm_period) {
    this->forward_pin = forward_pin;
    this->reverse_pin = reverse_pin;
    this->dir = FORWARD;
}

void DriveMotor::init() {
    PwmMotor::initPwm(this->forward_pin);
    PwmMotor::initPwm(this->reverse_pin);
}

// todo: comments
void DriveMotor::updateMotor(int pwm, Direction dir) {
    this->pwm = pwm;
    this->dir = dir;
}

// actuate updates the motors hardware to the set values
void DriveMotor::actuate() {
    if (dir == FORWARD) {
        writePwm(this->reverse_pin, 0);
        writePwm(this->forward_pin, this->pwm);
    }
    else {
        writePwm(this->reverse_pin, this->pwm);
        writePwm(this->forward_pin, 0);
    }
}
