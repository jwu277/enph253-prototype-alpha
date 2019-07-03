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
void DriveMotor::update(double pct) {

    int pwm = min((int) (fabs(pct) * this->pwm_period), this->pwm_period);
    this->pwm = pwm;

    this->dir = (pct >= 0.0) ? DriveMotor::FORWARD : DriveMotor::REVERSE;
    
}

// Actuates the motor...
void DriveMotor::actuate() {

    // IMPORTANT: ensure we never have both pins simeoultaneously on

    if (dir == DriveMotor::FORWARD) {
        writePwm(this->reverse_pin, 0);
        writePwm(this->forward_pin, this->pwm);
    }
    else {
        writePwm(this->forward_pin, 0);
        writePwm(this->reverse_pin, this->pwm);
    }
    
}
