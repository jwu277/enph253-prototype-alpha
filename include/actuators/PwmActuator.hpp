#ifndef PWMACTUATOR
#define PWMACTUATOR

#include "actuators/BaseActuator.hpp"

// PwmActuator is an abstract base class for actuators that are pwm controlled
class PwmActuator: public BaseActuator {

    protected:

         // Hz, the pwm clock frequency
        const int pwm_clk_freq;

        // # of clk periods, the period of a pwm cycle
        const int pwm_period;
    
        // The current pwm of the actuator, in # of clk periods
        int pwm;

        // Initializes the pwm clock frequency and period
        // clk_freq -- the pwm clock frequency in Hz
        // period   -- the pwm cycle period in # pwm clk cycles
        PwmActuator(int clk_freq, int period);

        // Initializes the pwm for a given pin in hardware and software
        // pin -- the pin to initialize pwm for
        void initPwm(PinName pin);

        // Sets the pwm for a given pin
        // pin  -- the pin to set the pwm for
        // duty -- the duty cycle to make the pwm, in # of clock cycles
        //         thus the duty cycle percentage is this divided by the period
        void writePwm(PinName pin, int duty);

};

#endif
