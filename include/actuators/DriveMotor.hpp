#ifndef MOTOR
#define MOTOR

#include "actuators\PolarityActuator.hpp"
#include "actuators\PwmActuator.hpp"

// DriveMotor is a concrete class that represents a driving motor.
class DriveMotor: public PolarityActuator, public PwmActuator {

    private:

        // The forward operation pin
        PinName forward_pin; 

        // The reverse operation pin
        PinName reverse_pin;   

    public:

        // Initializes the pins and pwm
        // forward_pin  -- the motor's forward operation pin
        // reverse_pin  -- the motor's reverse operation pin
        // pwm_clk_freq -- the pwm clock frequency in Hz
        // pwm_period   -- the pwm cycle period in # pwm clk cycles
        DriveMotor(PinName forward_pin, PinName reverse_pin, int pwm_clk_freq, int pwm_period);

        // Initializes the drive motor in HW
        void init();

        // Updates the motor
        // pct -- the percentage duty cycle to apply
        //     between -1.00 to 1.00 inclusive. Negative means reverse.
        //     Will be clipped to be within this range.
        void update(double pct);

        // Actuates the motor
        void actuate();

};

#endif
