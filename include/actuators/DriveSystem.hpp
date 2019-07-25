#ifndef DRIVESYSTEM
#define DRIVESYSTEM

#include <Arduino.h>

#include "actuators/BaseActuator.hpp"
#include "actuators/DriveMotor.hpp"


// DriveSystem is the driving system motors, composite of two DriveMotors
class DriveSystem: public BaseActuator {

    private:

        DriveMotor left_motor;
        DriveMotor right_motor;

        // to change BASE_DRIVE
        double speed_add;

    public:

        // Constructor
        DriveSystem(PinName left_motor_forward, PinName left_motor_reverse,
            PinName right_motor_forward, PinName right_motor_reverse,
            int pwm_clk_freq, int pwm_period);
        
        // HW init
        void init();

        // !! The last update before actuate() takes effect !!

        // Update motors in SW
        void update(double left_pct, double right_pct);

        // PID update with one variable
        void pid_update(double diff);

        // Actuate
        void actuate();

        void turn_left();

        void turn_right();

        void drive_forward();

        void set_speed_add(double val);

};

#endif
