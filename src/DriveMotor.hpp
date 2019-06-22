#ifndef MOTOR
#define MOTOR

#include "PolarityMotor.hpp"
#include "PwmMotor.hpp"

// TODO: refactor code

class DriveMotor: public PolarityMotor, public PwmMotor {

    private:

        //the pin that corresponds to driving the motor forward 
        PinName forward_pin; 

        //the pin that corresponds to driving the motor backward
        PinName reverse_pin;   

    public:

        //class constructor
        DriveMotor(PinName forward_pin, PinName reverse_pin, int pwm_clk_freq, int pwm_period);

        void init();

        // todo: comments
        void updateMotor(int pwm, Direction dir);

        // actuate updates the motors hardware to the set values
        void actuate();

};

#endif
