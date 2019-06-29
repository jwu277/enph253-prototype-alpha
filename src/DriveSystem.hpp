#ifndef DRIVESYSTEM
#define DRIVESYSTEM

#include <Arduino.h>

#include "BaseActuator.hpp"
#include "DriveMotor.hpp"

// DriveSystem is the driving system motors, composite of two DriveMotors
class DriveSystem: public BaseActuator {

    private:

        DriveMotor left_motor;
        DriveMotor right_motor;

    public:

        // Constructor
        DriveSystem(PinName left_motor_forward, PinName left_motor_reverse,
            PinName right_motor_forward, PinName right_motor_reverse,
            int pwm_clk_freq, int pwm_period);
        
        // HW init
        void init();

        // Update motors in SW
        void update(double left_pct, double right_pct);

        // PID update with one variable
        void pid_update(double diff);

        // Actuate
        void actuate();

};

#endif
