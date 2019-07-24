// Arduino Modules
#include <Arduino.h>
#include <PID_v1.h>

// Sensor Modules
#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"

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

// TODO

// (left to right)
vector<PinName> qrd_pins = {PA_7, PA_6, PA_5, PA_4};

vector<tuple<int, int>> qrd_calibration = {
    make_tuple(50, 150),
    make_tuple(50, 150),
    make_tuple(50, 150),
    make_tuple(50, 150)
};

vector<double> qrd_weights = {-1.5, -0.5, 0.5, 1.5}; // TODO

MainTapeSensor tape_sensor = MainTapeSensor(qrd_pins, qrd_calibration, qrd_weights);
//SideTapeSensor side_tape_sensor = SideTapeSensor(PA_5, PA_4); // TODO: pins

// Actuators
DriveSystem drive_system = DriveSystem(PB_9, PB_8, PB_6, PB_7, PWM_CLK_FREQ, PWM_PERIOD);

// Control/Logic/Computation

// PID
double* pid_input = tape_sensor.get_x_ptr();
double pid_output;
double pid_setpoint = 0.0;
PID drive_pid = PID(pid_input, &pid_output, &pid_setpoint, 0, 0, 0, DIRECT);;


IntersectionManager intersection_manager = IntersectionManager(
    &tape_sensor, &drive_system);

void setup() {

    // pinMode(PA_1, INPUT);
    // pinMode(PA_2, INPUT);

    //  double kp = (0.4 * analogRead(PA_6)) / 1024;
    //  double kd = (100.0 * analogRead(PA_7)) / 1024;
    double kp = 0.10977;
    double kd = 0.0;

    drive_pid = PID(pid_input, &pid_output, &pid_setpoint, kp, KI, kd, DIRECT);

    Serial.begin(9600);

    Serial.print(kp, 5);
    Serial.print("  ");
    Serial.print(kd, 5);
    Serial.println();

    // p 3.51562 d 0.00000
    //3.49609 3730.46875
    // p 0.01602 d 24.51172

    /*
    Serial.begin(9600);

    Serial.print("kp: ");
    Serial.print(kp, 3);
    Serial.println();

    Serial.print("kd: ");
    Serial.print(kd, 3);
    Serial.println();
    */

    init_sensors();
    init_actuators();
    init_logic();

    // Upper assembly
    
    attachInterrupt(ZHOME, zHomeISR, RISING);
    attachInterrupt(ZFULLEXT, zFullExtISR, RISING);
    attachInterrupt(YHOME, yHomeISR, RISING);
    attachInterrupt(YFULLEXT, yFullExtISR, RISING);
    attachInterrupt(CLAWPB, clawPBISR, RISING);
    attachInterrupt(CLAWFLOORPB, clawFloorPBISR, RISING);

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

    // pinMode(PA_1, INPUT);
    // pinMode(PA_3, INPUT);
    // pinMode(PA_7, INPUT);
    // pinMode(PA_6, INPUT);
    // pinMode(PA_5, INPUT);
    // pinMode(PA_4, INPUT);
    // pinMode(PA_0, INPUT);
    // pinMode(PA_2, INPUT);

    //
    
    // Hardware test
    //test_hardware();

    // drive_system.update(0.85, 0.85);
    // drive_system.actuate();
    // while(1);

       //grabCrystal();

    // homeY(true);
    // homeY(false);
    // while(1);

    digitalWrite(STEPPERENABLE, LOW);
    // while(1) {
    //     Serial.println("FOO");
    // }

    //delay(2000);

    // moveZToExtreme(EXTEND);
    // moveZToExtreme(HOME);

    //grabCrystal();

    // while (1) {
    //     Serial.print(analogRead(PA6));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA5));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA3));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA2));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA1));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA0));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA4));
    //     Serial.print("       ");
    //     Serial.print(analogRead(PA7));
    //     Serial.print("       ");
    //     Serial.println();
    // }

    // drive_system.update(1.0, 1.0);
    // drive_system.actuate();

    // while (1) {
    //     Serial.println("run2");
    // }

    // while (1) {

    //     drive_system.update(-3.0, -2.7);
    //     drive_system.actuate();

    //     delay(1000);

    // }

    // while(1) {
    //     Serial.println("running3");
    // }

    // while (1) {
    //     stepperPulse();
    //     delay(10);
    // }

    // //grabCrystal();

    // while(1) {
    //     Serial.println("code is running");
    //     delay(10);
    // }
    

}

void init_sensors() {

    tape_sensor.init();
    //side_tape_sensor.init();

}

void init_actuators() {

    drive_system.init();

}

void init_logic() {

    // PID
    drive_pid.SetOutputLimits(-2.0, 2.0);
    drive_pid.SetMode(AUTOMATIC);

}

//int foo = 0;
//long time;

void loop() {
    
    /*
    if (foo == 0) {
        time = millis();
    }*/
    //Serial.println("CANCER!");

    // TODO: incorporate interrupts

    // 1. Read new data from sensors
    update_sensors();

    // 2. Perform computations + update actuators in SW
    compute();

    // 3. Tick the actuators in HW
    run_actuators();
    
    //Serial.print("|");
    //Serial.print("       ");
    //Serial.print(*pid_input, 3);

    //Serial.println();

    
    //foo++;
    /*
    if (foo == 100) {
        Serial.print(millis() - time);
        Serial.println();
        foo = 0;
    }*/
    /*
    if (digitalRead(PA15)) {
        pwm_start(PB_4, 1000000, 10, 10, 0);
        pwm_start(PA_8, 1000000, 10, 10, 0);
    }
    else {
        pwm_start(PB_4, 1000000, 10, 0, 0);
        pwm_start(PA_8, 1000000, 10, 0, 0);
    }*/

    /*
    if (digitalRead(PA12)) {
        pwm_start(PA_8, 1000000, 10, 10, 0);
    }
    else {
        pwm_start(PA_8, 1000000, 10, 0, 0);
    }*/
    

}

void update_sensors() {

    tape_sensor.update();
    //side_tape_sensor.update();

}

void compute() {

    // TODO (ongoing): maybe organize computation logic into files
    // Use main_tape_sensor.x as PID input
    // Use drive_system.pid_update() as PID output

    drive_pid.Compute();
    // diff = -pid_output, since pid_output is negative of what we want
    drive_system.pid_update(-pid_output);

    //Serial.print(pid_output, 4);
    //Serial.println();

    intersection_manager.update();
//*pid_input
    
    if (tape_sensor.is_far_left()) {
        drive_system.update(0.74+pid_output*1.1, -2.9);
        //pwm_start(PA_0, 1000000, 10, 10, 0);
    }
    else {
        //pwm_start(PA_0, 1000000, 10, 0, 0);
    }

    if (tape_sensor.is_far_right()) {
        drive_system.update(-2.9, 0.76-pid_output*1.1);
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

    }
}
