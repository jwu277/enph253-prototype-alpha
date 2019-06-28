// Arduino Modules
#include <Arduino.h>

// Device Modules
#include <DriveMotor.hpp>

// Define Devices
DriveMotor left_drive_motor = DriveMotor(PA_0, PA_1, 1000, 100); // TODO: change


void setup() {
  // put your setup code here, to run once:
  left_drive_motor.init();
}

void loop() {
  // put your main code here, to run repeatedly:
}
