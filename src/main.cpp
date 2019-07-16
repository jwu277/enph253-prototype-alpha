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

// Constants
#define PWM_CLK_FREQ 10000000
#define PWM_PERIOD 1000

// PID Parameters
//#define KP 0.2
//#define KD 0.2
#define KI 0.01

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
DriveSystem drive_system = DriveSystem(PB_6, PB_7, PB_8, PB_9, PWM_CLK_FREQ, PWM_PERIOD);

// Control/Logic/Computation

// PID
double* pid_input = tape_sensor.get_x_ptr();
double pid_output;
double pid_setpoint = 0.0;
PID drive_pid = PID(pid_input, &pid_output, &pid_setpoint, 0, 0, 0, DIRECT);;


IntersectionManager intersection_manager = IntersectionManager(
    &tape_sensor, &drive_system);

void setup() {

    pinMode(PA_1, INPUT);
    pinMode(PA_2, INPUT);

    //  double kp = (0.1 * analogRead(PA_1)) / 1024;
    //  double kd = (100.0 * analogRead(PA_2)) / 1024;
    double kp = 0.01602;
    double kd = 24.51172;

    drive_pid = PID(pid_input, &pid_output, &pid_setpoint, kp, KI, kd, DIRECT);

    Serial.begin(9600);

    Serial.print(kp, 5);
    Serial.print(kd, 5);

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

    pwm_start(PB_4, 1000000, 10, 0, 1);
    pwm_start(PA_8, 1000000, 10, 0, 1);

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
    /*
    Serial.print(analogRead(PA_1));
    Serial.print("       ");
    Serial.print(analogRead(PA_3));
    Serial.print("       ");
    Serial.print(analogRead(PA_7));
    Serial.print("       ");
    Serial.print(analogRead(PA_6));
    Serial.print("       ");
    Serial.print(analogRead(PA_5));
    Serial.print("       ");
    Serial.print(analogRead(PA_4));
    Serial.print("       ");
    Serial.print(analogRead(PA_0));
    Serial.print("       ");
    Serial.print(analogRead(PA_2));
    Serial.println();*/
    

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
        drive_system.update(0.70+pid_output*1.1, -2.7);
        //pwm_start(PA_0, 1000000, 10, 10, 0);
    }
    else {
        //pwm_start(PA_0, 1000000, 10, 0, 0);
    }

    if (tape_sensor.is_far_right()) {
        drive_system.update(-2.7, 0.72-pid_output*1.1);
    }
   
    

}

void run_actuators() {

    drive_system.actuate();

}
