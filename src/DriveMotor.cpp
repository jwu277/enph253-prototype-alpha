#include <Arduino.h>
#include "DriveMotor.hpp"

// Initializes the pins and pwm
// forward_pin  -- the motor's forward operation pin
// reverse_pin  -- the motor's reverse operation pin
// pwm_clk_freq -- the pwm clock frequency in Hz
// pwm_period   -- the pwm cycle period in # pwm clk cycles
DriveMotor::DriveMotor(PinName forward_pin, PinName reverse_pin, int pwm_clk_freq, int pwm_period)
    : PwmActuator::PwmActuator(pwm_clk_freq, pwm_period) {
    this->forward_pin = forward_pin;
    this->reverse_pin = reverse_pin;
    this->dir = FORWARD;
}

// Initializes the drive motor in HW
void DriveMotor::init() {
    PwmActuator::initPwm(this->forward_pin);
    PwmActuator::initPwm(this->reverse_pin);
}

// Updates the motor
// TODO: look into moving this into BaseActuator
void DriveMotor::update(int pwm, Direction dir) {
    this->pwm = pwm;
    this->dir = dir;
}

// Actuates the motor...
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
