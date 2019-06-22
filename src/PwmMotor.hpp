#ifndef PWMMOTOR
#define PWMMOTOR

#include "BaseActuator.hpp"

class PwmMotor: public BaseActuator {

    protected:
    
        //the pwm that the motor travels at. An integer between 0-255
        int pwm;

        const int pwm_clk_freq; // Hz
        const int pwm_period; // # of clk periods

        PwmMotor(int clk_freq, int period);

        // TODO: doc
        void initPwm(PinName pin);

        //TODO: doc (+ note guard)
        void writePwm(PinName pin, int duty);

};

#endif
