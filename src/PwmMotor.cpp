#include <Arduino.h>
#include "PwmMotor.hpp"

PwmMotor::PwmMotor(int clk_freq, int period) : pwm_clk_freq(clk_freq), pwm_period(period) {
    this->pwm = 0;
}

void PwmMotor::initPwm(PinName pin) {
    pwm_start(pin, pwm_clk_freq, pwm_period, 0, 1);
}

//TODO: doc (+ note guard)
void PwmMotor::writePwm(PinName pin, int duty) {
    pwm_start(pin, pwm_clk_freq, pwm_period, fmax(0,fmin(duty, pwm_period)), 0);
}
