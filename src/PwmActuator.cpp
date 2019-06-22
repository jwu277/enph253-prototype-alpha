#include <Arduino.h>
#include "PwmActuator.hpp"

// Initializes the pwm clock frequency and period
// clk_freq -- the pwm clock frequency in Hz
// period   -- the pwm cycle period in # pwm clk cycles
PwmActuator::PwmActuator(int clk_freq, int period) : pwm_clk_freq(clk_freq), pwm_period(period) {
    this->pwm = 0;
}

// Initializes the pwm for a given pin in hardware and software
// pin -- the pin to initialize pwm for
void PwmActuator::initPwm(PinName pin) {
    pwm_start(pin, pwm_clk_freq, pwm_period, 0, 1);
}

// Sets the pwm for a given pin
// pin  -- the pin to set the pwm for
// duty -- the duty cycle to make the pwm, in # of clock cycles
//         thus the duty cycle percentage is this divided by the period
void PwmActuator::writePwm(PinName pin, int duty) {
    pwm_start(pin, pwm_clk_freq, pwm_period, fmax(0,fmin(duty, pwm_period)), 0);
}
