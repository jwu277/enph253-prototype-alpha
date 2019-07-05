#include <Arduino.h>

#include "DriveSystem.hpp"

// Base driving pwm for motors
#define DIFFERENTIAL_BASE 0.78


// Constructor
DriveSystem::DriveSystem(PinName left_motor_forward, PinName left_motor_reverse,
    PinName right_motor_forward, PinName right_motor_reverse,
    int pwm_clk_freq, int pwm_period)
    : left_motor(left_motor_forward, left_motor_reverse, pwm_clk_freq, pwm_period),
    right_motor(right_motor_forward, right_motor_reverse, pwm_clk_freq, pwm_period) {

}

void DriveSystem::init() {
    this->left_motor.init();
    this->right_motor.init();
}

// Read data
void DriveSystem::update(double left_pct, double right_pct) {

    Serial.print("Left: ");
    Serial.print(left_pct, 3);
    Serial.println();

    Serial.print("Right: ");
    Serial.print(right_pct, 3);
    Serial.println();

    this->left_motor.update(left_pct);
    this->right_motor.update(right_pct);

}

// diff -- motor differential, steers
//     +ve means turn left (right motor more pwm), -ve opposite
void DriveSystem::pid_update(double diff) {
    this->update(1.0, 0.8);
    //Serial.print(diff, 3);
    //Serial.println();
    //this->update(DIFFERENTIAL_BASE - diff, DIFFERENTIAL_BASE + diff);
}

void DriveSystem::actuate() {
    this->left_motor.actuate();
    this->right_motor.actuate();
}
