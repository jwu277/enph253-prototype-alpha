// Arduino Modules
#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>

// Sensor Modules
#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"
#include "sensors/MPU6050.h"
#include "sensors/I2Cdev.h"

// Actuator Modules
#include "actuators/DriveSystem.hpp"

// Control/Logic/Computation Classes
#include "logic/IntersectionManager.hpp"

// Upper assembly (TODO: organize)
#include "claw_system.h"

// Constants
#define PWM_CLK_FREQ 10000000
#define PWM_PERIOD 1000

// PID Parameters
//#define KP 0.2
//#define KD 0.2
#define KI 0.02

using namespace std;

// Function declarations

// Setup
void init_sensors();
void init_actuators();
void init_logic();

// Loop
void update_sensors();
void compute();
void run_actuators();

// Sensors

// QRD Tape Sensor System
// (on_tape, on_white)

// Testing only
void test_hardware();

MainTapeSensor tape_sensor = MainTapeSensor();

// Actuators
DriveSystem drive_system = DriveSystem(PB_6, PB_7, PB_9, PB_8, PWM_CLK_FREQ, PWM_PERIOD);

// Control/Logic/Computation

// PID
double* pid_input = tape_sensor.get_x_ptr();
double pid_output;
double pid_setpoint = 0.0;
PID drive_pid = PID(pid_input, &pid_output, &pid_setpoint, 0, 0, 0, DIRECT);;


IntersectionManager intersection_manager = IntersectionManager(
    &tape_sensor, &drive_system);

// Accelerometer
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define GRAVITY 9.81
#define CONVERSION_FACTOR GRAVITY / 2048 // For +- 16g reading
#define ACCEL_DEBOUNCE 300 // ms
long accel_trigger_time = millis();

void setup() {

    //TUNING PID
    //  double kp = (0.4 * analogRead(PA_6)) / 1024;
    //  double kd = (100.0 * analogRead(PA_7)) / 1024;
    double kp = 0.10977;
    double kd = 0.0;

    drive_pid = PID(pid_input, &pid_output, &pid_setpoint, kp, KI, kd, DIRECT);

    Serial.begin(9600);

    // record pid tuning parameters 
    // Serial.print(kp, 5);
    // Serial.print("  ");
    // Serial.print(kd, 5);
    // Serial.println();

    init_sensors();
    init_actuators();
    init_logic();

    // Upper assembly
    
    attachInterrupt(ZHOME, zHomeISR, RISING);
    attachInterrupt(ZFULLEXT, zFullExtISR, RISING);
    attachInterrupt(YHOME, yHomeISR, RISING);
    attachInterrupt(YFULLEXT, yFullExtISR, RISING);
    attachInterrupt(CLAWPB, clawPBISR, RISING);
    attachInterrupt(CLAWFLOORPB, clawFloorPBISR, FALLING);

    //stepper
    pinMode(CLAWSERVO, OUTPUT);
    pinMode(YSERVO, OUTPUT);
    pinMode(STEPPERCLK, OUTPUT);
    pinMode(STEPPERSLEEP, OUTPUT);
    pinMode(STEPPERDIR, OUTPUT);
    pinMode(STEPPERENABLE, OUTPUT);
    pinMode(ZFULLEXT, INPUT);
    pinMode(ZHOME, INPUT);
    pinMode(YHOME, INPUT);
    pinMode(YFULLEXT, INPUT);
    pinMode(CLAWPB, INPUT);
    pinMode(CLAWFLOORPB, INPUT);

    digitalWrite(STEPPERENABLE, HIGH);
    digitalWrite(STEPPERSLEEP, HIGH);
    digitalWrite(STEPPERDIR, UP);
    digitalWrite(STEPPERCLK, LOW);

    closeClaw();


    // Hardware test
    //test_hardware();

    // I2C for accelerometer
    Wire.setSDA(PB11);
    Wire.setSCL(PB10);
    Wire.begin();

    accelgyro.initialize();

    // Set accelerometer range to be +- 16g
    accelgyro.setFullScaleAccelRange(3);

}

void init_sensors() {
    tape_sensor.init();
}

void init_actuators() {
    drive_system.init();
}

void init_logic() {
    // PID
    drive_pid.SetOutputLimits(-2.0, 2.0);
    drive_pid.SetMode(AUTOMATIC);
}

void loop() {
    // 1. Read new data from sensors
    update_sensors();

    // 2. Perform computations + update actuators in SW
    compute();

    // 3. Tick the actuators in HW
    run_actuators();
    
}

void update_sensors() {
    tape_sensor.update();
}

void compute() {

    // TODO (ongoing): maybe organize computation logic into files
    // Use main_tape_sensor.x as PID input
    // Use drive_system.pid_update() as PID output

    drive_pid.Compute();
    // diff = -pid_output, since pid_output is negative of what we want
    drive_system.pid_update(-pid_output);

    intersection_manager.update();
    
    if (tape_sensor.is_far_left()) {
        drive_system.update(0.74+pid_output*1.1, -2.9);
    }
    if (tape_sensor.is_far_right()) {
        drive_system.update(-2.9, 0.76-pid_output*1.1);
    }

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Serial.println((ax * ax + ay * ay) * CONVERSION_FACTOR * CONVERSION_FACTOR);

    if (millis() - accel_trigger_time >= ACCEL_DEBOUNCE) {
        if (fabs(ax) * CONVERSION_FACTOR >= 8 || fabs(ay) * CONVERSION_FACTOR >= 12) {
            // TODO: collision handling

            // Serial.println("BUMP");
            // Serial.println(ax * CONVERSION_FACTOR);
            // Serial.println(ay * CONVERSION_FACTOR);
            // Serial.println();
            accel_trigger_time = millis();
        }
    }

}

void run_actuators() {
    drive_system.actuate();
}

// Just to test
void test_hardware() {
    // Only runs this unless commented out
    while (true) {

        // 1. Blink LED 3 times
        // pwm_start(PB_4, 1000000, 10, 10, 0);
        // delay(300);
        // pwm_start(PB_4, 1000000, 10, 0, 0);
        // delay(300);
        // pwm_start(PB_4, 1000000, 10, 10, 0);
        // delay(300);
        // pwm_start(PB_4, 1000000, 10, 0, 0);
        // delay(300);
        // pwm_start(PB_4, 1000000, 10, 10, 0);
        // delay(300);
        // pwm_start(PB_4, 1000000, 10, 0, 0);
        // delay(300);

        delay(1500);


            Serial.print(analogRead(PA6));
            Serial.print("       ");
            Serial.print(analogRead(PA5));
            Serial.print("       ");
            Serial.print(analogRead(PA3));
            Serial.print("       ");
            Serial.print(analogRead(PA2));
            Serial.print("       "); 
            Serial.print(analogRead(PA1));
            Serial.print("       ");
            Serial.print(analogRead(PA0));
            Serial.print("       ");
            Serial.print(analogRead(PA4));
            Serial.print("       ");
            Serial.print(analogRead(PA7));
            Serial.print("       ");
            Serial.println();


        // 2. Check that QRDs are white
        if (fmax(fmax(fmax(fmax(fmax(fmax(fmax(analogRead(PA_1),
            analogRead(PA_3)), analogRead(PA_7)), analogRead(PA_6)),
            analogRead(PA_5)), analogRead(PA_4)), analogRead(PA_0)),
            analogRead(PA_2)) >= 150) {
            
            //pwm_start(PB_4, 1000000, 10, 10, 0);

            Serial.println("bad");

        }
        else {
            //pwm_start(PB_4, 1000000, 10, 0, 0);
            Serial.println("NICE!");
        }

        // Keep LED in this state until test reuns

        // 3. Run motors
        drive_system.update(0.9, 0.4); // Right forward
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);
        drive_system.update(-2.7, -1.2); // Right reverse
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);
        drive_system.update(0.4, 0.9); // Left forward
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);
        drive_system.update(-1.2, -2.7); // Left reverse
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);
        drive_system.update(0.85, 0.85); // Straight forward
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);
        drive_system.update(-2.5, -2.5); // Straight reverse
        drive_system.actuate();
        delay(1000);
        drive_system.update(0.0, 0.0);
        drive_system.actuate();
        delay(300);

        moveZToExtreme(EXTEND);
        homeY(true);
        homeY(false);
        moveZToExtreme(HOME);

        openClaw();
        closeClaw();

    }
}
