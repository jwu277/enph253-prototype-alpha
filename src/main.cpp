// Arduino Modules
#include <Arduino.h>
#include <PID_v1.h>

// Sensor Modules
#include "MainTapeSensor.hpp"
#include "SideTapeSensor.hpp"

// Actuator Modules
#include "DriveSystem.hpp"

// Constants
#define PWM_CLK_FREQ 10000000
#define PWM_PERIOD 1000

// PID Parameters
<<<<<<< HEAD
#define KP 17.0
=======
#define KP 16.0
>>>>>>> db5f3532f782da575744932754d73114e0a4605f
#define KD 16.0
#define KI 0.0

// Function declarations
void sensor_init();
void actuator_init();

void update_sensors();
void compute();
void run_actuators();

// Create + SW Init Devices
// TODO: pins

// Sensors
MainTapeSensor main_tape_sensor = MainTapeSensor(PA_7, PA_6);
SideTapeSensor side_tape_sensor = SideTapeSensor(PA_12, PA_13); // TODO: pins

// Actuators
DriveSystem drive_system = DriveSystem(PB_6, PB_7, PB_8, PB_9, PWM_CLK_FREQ, PWM_PERIOD);

// Control
double* pid_input = main_tape_sensor.get_x_ptr();
double pid_output;
double pid_setpoint = 0.0;
PID drive_pid = PID(pid_input, &pid_output, &pid_setpoint, KP, KI, KD, DIRECT);

void setup() {

<<<<<<< HEAD
  sensor_init();
=======
  //Serial.begin(9600);
>>>>>>> db5f3532f782da575744932754d73114e0a4605f
  
  actuator_init();

  drive_pid.SetOutputLimits(-2.0, 2.0);
  drive_pid.SetMode(AUTOMATIC);

}

void sensor_init() {

  main_tape_sensor.init();
  side_tape_sensor.init();

}

void actuator_init() {

  drive_system.init();

}

void loop() {

  // TODO: incorporate interrupts

  // 1. Read new data from sensors
  update_sensors();

  // 2. Perform computations + update actuators in SW
  compute();

  // 3. Tick the actuators in HW
  run_actuators();

}

void update_sensors() {

  main_tape_sensor.update();

}

void compute() {

  // TODO: maybe organize computation logic into files
  // Use main_tape_sensor.x as PID input
  // Use drive_system.pid_update() as PID output

  Serial.print(*pid_input, 3);
  Serial.println();

  drive_pid.Compute();
  drive_system.pid_update(pid_output);

}

void run_actuators() {

  drive_system.actuate();

}
