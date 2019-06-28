// Arduino Modules
#include <Arduino.h>

// Device Modules
#include "DriveMotor.hpp"
#include "QrdSensor.hpp"

// Constants
#define PWM_CLK_FREQ 1000000
#define PWM_PERIOD 100

// Functions
void actuator_init();

void read_sensors();
void compute();
void update_actuators();
void run_actuators();

// Create + SW Init Devices
// TODO: pins
DriveMotor left_drive_motor = DriveMotor(PA_0, PA_1, PWM_CLK_FREQ, PWM_PERIOD);
DriveMotor right_drive_motor = DriveMotor(PA_2, PA_3, PWM_CLK_FREQ, PWM_PERIOD);

QrdSensor left_tape_sensor = QrdSensor(PA_4);
QrdSensor right_tape_sensor = QrdSensor(PA_5);

void setup() {
  
  actuator_init();

}

void actuator_init() {

  left_drive_motor.init();
  right_drive_motor.init();

}

void loop() {

  // TODO: incorporate interrupts

  // 1. Read new data from sensors
  read_sensors();

  // TODO: maybe merge compute() with update_actuators() ?

  // 2. Perform computations
  compute();

  // 3. Update actuators in SW
  update_actuators();

  // 4. Tick the actuators in HW
  run_actuators();

}

void read_sensors() {

}

void compute() {

}

void update_actuators() {

}

void run_actuators() {

  left_drive_motor.actuate();
  right_drive_motor.actuate();

}
