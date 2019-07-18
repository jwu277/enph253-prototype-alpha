#include <Arduino.h>

#include "actuators/DriveSystem.hpp"

// Base driving pwm for motors
#define BASE_DRIVE 0.90

#define TURN_LOW 0.5
#define TURN_HIGH 0.98

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

    /*
    Serial.print("Left: ");
    Serial.print(left_pct, 3);
    Serial.println();

    Serial.print("Right: ");
    Serial.print(right_pct, 3);
    Serial.println();
    */

    this->left_motor.update(left_pct);
    this->right_motor.update(right_pct);

}

// diff -- motor differential, steers
//     +ve means turn left (right motor more pwm), -ve opposite
void DriveSystem::pid_update(double diff) {
    //this->update(0.8, 1.0);

    if (diff >= 0) {
        this->update(BASE_DRIVE - diff, BASE_DRIVE + diff * 0.3);
    }
    else {
        this->update(BASE_DRIVE - diff * 0.3, BASE_DRIVE + diff);
    }

}

void DriveSystem::actuate() {
    this->left_motor.actuate();
    this->right_motor.actuate();
}

void DriveSystem::turn_left() {
    this->update(TURN_LOW, TURN_HIGH);
}

void DriveSystem::turn_right() {
    this->update(TURN_HIGH, TURN_LOW);
}

void DriveSystem::drive_forward() {
    this->update(BASE_DRIVE, BASE_DRIVE);
}
