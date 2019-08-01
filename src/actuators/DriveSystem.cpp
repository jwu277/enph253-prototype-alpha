#include <Arduino.h>

#include "actuators/DriveSystem.hpp"

// Base driving pwm for motors
#define BASE_DRIVE 0.92

// Constructor
DriveSystem::DriveSystem(PinName left_motor_forward, PinName left_motor_reverse,
    PinName right_motor_forward, PinName right_motor_reverse,
    int pwm_clk_freq, int pwm_period)
    : left_motor(left_motor_forward, left_motor_reverse, pwm_clk_freq, pwm_period),
    right_motor(right_motor_forward, right_motor_reverse, pwm_clk_freq, pwm_period) {
    this->speed_add = 0.0;
}

void DriveSystem::init() {
    this->left_motor.init();
    this->right_motor.init();
}

// Read data
void DriveSystem::update(double left_pct, double right_pct) {
    this->left_motor.update(left_pct);
    this->right_motor.update(right_pct);
}

// diff -- motor differential, steers
//     +ve means turn left (right motor more pwm), -ve opposite
void DriveSystem::pid_update(double diff) {
    if (diff >= 0) {
        this->update(BASE_DRIVE - diff + this->speed_add, BASE_DRIVE + diff * 0.3 + this->speed_add);
    }
    else {
        this->update(BASE_DRIVE - diff * 0.3 + this->speed_add, BASE_DRIVE + diff + this->speed_add);
    }
}

void DriveSystem::actuate() {
    this->left_motor.actuate();
    this->right_motor.actuate();
}

void DriveSystem::drive_forward() {
    this->update(BASE_DRIVE, BASE_DRIVE);
}

void DriveSystem::set_speed_add(double val) {
    this->speed_add = val;
}
