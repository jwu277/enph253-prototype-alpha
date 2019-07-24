#include <Arduino.h>

#include "claw_system.h"

//crane pushbutton booleans
volatile bool zIsHome = 0;
volatile bool yIsHome = 0;
volatile bool zIsExtended = 0;
volatile bool yIsExtended = 0;
volatile bool clawPBPressed = 0;
volatile bool clawBasePBPressed = 0;

//timer ISR variables
volatile byte motTimeControl; //a byte consisting of flags for the timerISR to use in processing
volatile int stepsLeft;       //the number of steps left for the stepper motor to move
volatile int zTimeInc;        //the number of z time increments that have passed since the last variable reset
volatile int yTimeInc;        //the number of y time increments passed since the last variable reset
volatile int clawTimeInc;
volatile int timePerInc;     //the software defined time interval of the timer interrupt
const int timePerStep = 0;   //the user defined time per step, gives the speed of the stepper motor
const int totalYTime = 0;    //the user defined time per step, gives the speed of the stepper motor
const int totalClawTime = 0; //the user defined time per step, gives the speed of the stepper motor

int globalcount = 0;
//global status variables
volatile double yPos = 0.0;
volatile int yStatus = NOT_MOVING;
volatile int clawStatus = NOT_MOVING;
volatile bool clawIsOpen = false;

void zHomeISR()
{
    //Serial.println("zhome");
    zIsHome = true;
}

void zFullExtISR()
{
    //Serial.println("zFullExtISR");
    zIsExtended = true;
}

void yHomeISR()
{
    //Serial.println("yHomeISR");
    if (yStatus == MOVING_BK)
    {
        pwm_stop(Y_SERVO_PWM_NAME);
        yStatus = NOT_MOVING;
    }
    //Serial.println("Y=0 limit reached");
    yPos = 0;
    yIsHome = true;
    yIsExtended = false;
}

void yFullExtISR()
{
    //Serial.println("yFullExtISR");
    if (yStatus == MOVING_FWD)
    {
        pwm_stop(Y_SERVO_PWM_NAME);
        yStatus = NOT_MOVING;
    }
    //Serial.println("Y max limit reached");
    yPos = 300;
    yIsExtended = true;
    yIsHome = false;
}

void clawPBISR()
{
    //Serial.println("clawPBISR");
    clawPBPressed = true;
}

void clawFloorPBISR()
{
    //Serial.println("clawFloorPBISR");
    clawBasePBPressed = true;
}

void openClaw()
{
    if (!clawIsOpen)
    {
        //Serial.println("Opening claw");
        pwm_start(
            CLAW_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            FWD_PERIOD,
            FWD_ON_PERIOD,
            1);
        clawStatus = MOVING_FWD;
        //Serial.println("PWM to open claw started");

        delay(2000);

        pwm_stop(CLAW_SERVO_PWM_NAME);
    }
    clawStatus = NOT_MOVING;
    clawIsOpen = true;
}

void closeClaw()
{
    if (clawIsOpen)
    {
        //Serial.println("Closing claw");
        pwm_start(
            CLAW_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            BK_PERIOD,
            BK_ON_PERIOD,
            1);
        // Serial.println("PWM to close claw started");
        clawStatus = MOVING_BK;
        // Serial.println("PWM to open claw started");

        delay(2500);

        pwm_stop(CLAW_SERVO_PWM_NAME);
    }
    clawStatus = NOT_MOVING;
    clawIsOpen = false;
}

void homeY(bool retract)
{
    unsigned long timeoutStart = millis();
    unsigned long timeoutNow = millis();

    // Serial.println("Starting Y home");
    yIsHome = digitalRead(YHOME);
    yIsExtended = digitalRead(YFULLEXT);
    if (!yIsHome && retract)
    {
        // Serial.println("Y not home");
        pwm_start(
            Y_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            BK_PERIOD,
            BK_ON_PERIOD,
            1);
        yStatus = MOVING_BK;
        // Serial.println("Y homing started");
        while (!yIsHome || timeoutNow - timeoutStart >= Y_HOME_TIMEOUT)
        {
            timeoutNow = millis();
        }
        timeoutNow = millis();
        // Serial.print("ms for home operation: ");
        // Serial.println(timeoutNow - timeoutStart);
    }
    else if (!yIsExtended && !retract)
    {
        // Serial.println("Y not extended");
        pwm_start(
            Y_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            FWD_PERIOD,
            FWD_ON_PERIOD,
            1);
        yStatus = MOVING_FWD;
        // Serial.println("Y extension started");
        while (!yIsExtended || timeoutNow - timeoutStart >= Y_HOME_TIMEOUT)
        {
            timeoutNow = millis();
        }
        timeoutNow = millis();
        // Serial.print("ms for extend operation: ");
        // Serial.println(timeoutNow - timeoutStart);
    }
    else
    {
        // Serial.println("Invalid conditions for this home operation");
        pwm_stop(Y_SERVO_PWM_NAME);
    }
}

void moveY(double dist)
{
    double timeToRun = 0;
    if (dist < 0)
    {
        timeToRun = abs(dist) * (60000 / (134.05 * 62.832));
        pwm_start(
            Y_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            BK_PERIOD,
            BK_ON_PERIOD,
            1);
        yStatus = MOVING_BK;
    }
    else if (dist > 0)
    {
        timeToRun = abs(dist) * (60000 / (129.05 * 62.832));
        pwm_start(
            Y_SERVO_PWM_NAME,
            PWM_CLOCK_FREQ,
            FWD_PERIOD,
            FWD_ON_PERIOD,
            1);
        yStatus = MOVING_FWD;
    }
    //   Serial.print("Starting PWM for time = ");
    //   Serial.println(timeToRun);
    delay(timeToRun);
    pwm_stop(Y_SERVO_PWM_NAME);
    yStatus = NOT_MOVING;
    yPos += dist;
}

void moveYUntilClawPressed()
{

    clawPBPressed = false;

    pwm_start(
                Y_SERVO_PWM_NAME,
                PWM_CLOCK_FREQ,
                FWD_PERIOD,
                FWD_ON_PERIOD,
                1);
    yStatus = MOVING_FWD;

    int run_time = 0;

    while(!clawPBPressed) {  
        run_time++;
        delay(1);
    }

    clawPBPressed = false;
    pwm_stop(Y_SERVO_PWM_NAME);
    yStatus = NOT_MOVING;
    yPos += run_time * (129.05 * 62.832) / 60000;
}

void grabCrystal()
{
    digitalWrite(STEPPERENABLE, LOW);
    //homeY(true);
    moveZToExtreme(EXTEND);
    // while(1) {
    //     Serial.println("running");
    // }
    openClaw();
    moveY(200);
    findTopOfPillar();
    moveYUntilClawPressed();
    closeClaw();
    moveZToExtreme(EXTEND);
    homeY(true);
    moveZToExtreme(HOME);
    openClaw();
    digitalWrite(STEPPERENABLE, HIGH);
}

void findTopOfPillar()
{
    clawBasePBPressed = false;
    if (digitalRead(CLAWFLOORPB))
        return;
    changeStepperDir(DOWN);
    while (!clawBasePBPressed)
    {
        stepperPulse();
    }
    clawBasePBPressed = false;
}

void moveZToExtreme(bool home)
{
    //set the home flags to false to allow for the positive edge of the interrupts to cause a rising flag

    zIsHome = 0;
    zIsExtended = 0;

    if (home == true)
    {
        if (digitalRead(ZHOME))
            return; //if it is sensed that z is home, quit this protocal since a rising edge interrupt can not occur
        digitalWrite(STEPPERDIR, DOWN);
        while (!zIsHome)
        {
            stepperPulse();
        }
        zIsHome = false; //reset the flag
    }
    else
    {
        // if (digitalRead(ZFULLEXT)) {
        //     while (1) {
        //         Serial.println("FUCK");
        //         delay(10);
        //     }
        //     return; //if it is sensed that z is extended, quit this protocal since a rising edge interrupt can not occur
        // }
        digitalWrite(STEPPERDIR, HIGH);
        while (!zIsExtended)
        {
            //Serial.println("HI");
            stepperPulse();
        }
        zIsExtended = false; //reset the flag
    }
}

void changeStepperDir(bool dir)
{
    digitalWrite(STEPPERDIR, dir);
}

void enableStepper()
{
    digitalWrite(STEPPERCLK, LOW);
}

void disableStepper()
{
    digitalWrite(STEPPERCLK, HIGH);
}
/* 
StepperPulse sends a pwm signal to the CLK pin of a stepper motor driver
 */
void stepperPulse()
{
    digitalWrite(STEPPERCLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEPPERCLK, LOW);
    delayMicroseconds(2600);
}

//moves the z axis for "steps" steps
void moveZSteps(int steps, bool dir)
{
    if (dir == UP)
    {
        changeStepperDir(UP);
        for (int i = 0; i < steps; i++)
        {
            if (zIsExtended)
            {
                zIsExtended = false;
                return;
            }
            stepperPulse();
        }
    }
    else
    {
        changeStepperDir(DOWN);
        for (int i = 0; i < steps; i++)
        {
            if (zIsHome)
            {
                zIsHome = false;
                return;
            }
            stepperPulse();
        }
    }
}

int mmToSteps(int mm)
{
    float r = 10;                                      //radius of belt holder in mm
    float stepsPermm = (float)200 / (2 * 3.14159 * r); //200 stepsPerrotation / (lenghtPerrotation) gives steps per length mm
    return (int)((float)mm * stepsPermm);
}
