#include <Arduino.h>

//home and limiting digital switches
#define ZHOME PB5
#define ZFULLEXT PB4
#define YHOME PA8
#define YFULLEXT PB3
#define CLAWPB PB14
#define CLAWFLOORPB PB13

//Servos
#define CLAWSERVO PB1
#define CLAW_SERVO_PWM_NAME PB_1
#define YSERVO PB0
#define Y_SERVO_PWM_NAME PB_0

//Stepper motors
#define STEPPERENABLE PB12
#define STEPPERDIR PA11
#define STEPPERCLK PA15
#define STEPPERSLEEP PA12

//boolean definitions
#define UP 1
#define DOWN 0
#define X 1
#define Y 0
#define HOME 1
#define EXTEND 0

//operational time constants (generally in ms)
#define Y_HOME_TIMEOUT 5000
#define CLAW_ON_DURATION 2000

//state definitions
#define NOT_MOVING 0
#define MOVING_FWD 1
#define MOVING_BK 2

//PWM constants
#define PWM_CLOCK_FREQ 100000 //Hz
#define FWD_PERIOD 2170
#define FWD_ON_PERIOD 170
#define BK_PERIOD 2135
#define BK_ON_PERIOD 135

void hardwareISR();
void zHomeISR();
void yHomeISR();
void zFullExtISR();
void yFullExtISR();
void clawPBISR();
void clawFloorPBISR();
void servoPulseShort(byte);
void servoPulseLong(byte);
void stepperPulse(int);
void homeY(bool retract);
void moveY(double dist);
void closeClaw();
void openClaw();
int moveZToExtreme(bool, int);
void changeStepperDir(bool);
void findTopOfPillar(int);
bool grabCrystal(int);
void depositCrystal();
void enableStepper();
void disableStepper();
int moveZSteps(int, bool, int);
int mmToSteps(int);
void moveYUntilClawPressed();
int moveZDist(bool, int, int);
