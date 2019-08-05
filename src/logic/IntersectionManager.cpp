#include <Arduino.h>

#include "sensors/MainTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include "logic/IntersectionManager.hpp"

#include "claw_system.h"

#define TURN_COUNTER_MAX 100
#define DELAY_TIME 500

#define REVERSE_LEFT false
#define REVERSE_RIGHT true

#define Y_ISECT true
#define T_ISECT false

#define SERIAL_ISECT_RETRIES 10

unsigned long last_intersection_time = millis();

unsigned long gauntlet_timer;

bool handling_gauntlet = false;

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* tape_sensor,
    DriveSystem* drive_system) {
        
    // Modules
    this->tape_sensor = tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

    this->gauntlet_state = 0;

    this->initialize_tasksToDo();

    this->isectType = T_ISECT; // why this init value?

    this->task = TASK_R;

}

void IntersectionManager::set_side(bool side) {
    this->side = side;
}

void IntersectionManager::wiggle(int numOfWiggles, int wiggleHalfPeriod) {
    for (int i = 0; i < numOfWiggles; i++) {
        this->drive_system->update(0.93, -0.1);
        this->drive_system->actuate();
        delay(wiggleHalfPeriod);
        this->drive_system->update(-0.1, 0.93);
        this->drive_system->actuate();
        delay(wiggleHalfPeriod);
    }
}

void IntersectionManager::reverseAndTurn(int reverseTime, int turnTime, bool dir) {
    //if dir is true, turns right
    this->drive_system->update(-3.0, -3.0);
    this->drive_system->actuate();
    delay(reverseTime);
    if (dir == REVERSE_RIGHT) {
        this->drive_system->update(.98, -3.0);
        this->drive_system->actuate();
        delay(turnTime);
        this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
    }
    else {
        this->drive_system->update(-3.0, .98);
        this->drive_system->actuate();
        delay(turnTime);
        this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
    }    
}

void IntersectionManager::motorsOff(int duration) {
    this->drive_system->update(0, 0);
    this->drive_system->actuate();
    delay(duration);
}

bool IntersectionManager::readSerialIsectType() {
    return Y_ISECT;
    Serial.println("~");
    for (int i = 0; i < SERIAL_ISECT_RETRIES; i++) {
        delay(5);
        if (Serial.available()) {
            if (Serial.read() == 'Y') {
                Serial.println("Received Y classification");
                return Y_ISECT;
            }
            else if (Serial.read() == 'T') {
                Serial.println("Received T classification");
                return T_ISECT;
            }
        }
    }
    // todo have a fail state
    Serial.println("Classify timed out. Returning T");
    return T_ISECT;
}

// Exectutes Uturn in rotating clockwise for direction = true and counter clockwise otherwise. 
void IntersectionManager::uturn(bool dir) {
    unsigned long start_uturn_time = millis();
    //if dir is true, turns right
    this->drive_system->update(-3.0, -3.0);
    this->drive_system->actuate();
    delay(100);
    if (dir == true) {
        this->drive_system->update(.98, -3.3);
        this->drive_system->actuate();
        delay(300);
        this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
    }
    else {
        this->drive_system->update(-3.3, .98);
        this->drive_system->actuate();
        delay(300);
        this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
    } 

    this->motorsOff(6969669);

}

void IntersectionManager::update() {


    // temp: handle gauntlet
    // TODO: manage intersection increments
    // if (this->intersection_count == 7) {
    //     this->handle_gauntlet();
    // } 

    if (this->at_t_intersection() || this->at_y_intersection() || handling_gauntlet) {

        unsigned long new_time = millis();
        if (handling_gauntlet) {
            this->handle_intersection();
        }
        //Debounce in case intersection triggered by accident due to oscilation 
        else if (new_time - last_intersection_time >= DELAY_TIME) {
            Serial.println("At intersection");
            this->handle_intersection();
            last_intersection_time = new_time;
        }
        else {
            last_intersection_time = millis();
        }
    }
}
//Y intersection defined as at least 2 subsequent black with at least one white followed by at least 2 subsequent black
bool IntersectionManager::at_y_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    // duplicate end elements for correct detection
    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);

    bool cond = false;

    it = qrds_status.begin();

    // Search for two blacks in a row
    int count = 0;
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            count++;
            if (count == 1) {
                break;
            }
        }
    }

    // Skip whites + ensure at least one white
    count = 0;
    for (; it != qrds_status.end(); it++) {
        if(!*it) {
            count++;
        }
        else if (count >= 1) {
            break;
        }
    }
    // Search for two blacks in a row
    count = 0;
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            count++;
            if (count == 1) {
                cond = true;
                break;
            }
        }
    }

    it = qrds_status.begin();
    bool val1 = *it;
    advance(it, 2);
    bool val2 = *it;

    it = qrds_status.end();
    bool val3 = *it;
    advance(it, -2);
    bool val4 = *it;

    bool cond2 = (val1 && val2) || (val3 && val4);

    return cond || cond2;

}

// Only require 1 black on both sides of wide
bool IntersectionManager::at_y_intersection_lenient() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    // duplicate end elements for correct detection
    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);

    bool cond = false;

    it = qrds_status.begin();

    // Search for two blacks in a row
    int count = 0;
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            count++;
            if (count == 1) {
                break;
            }
        }
    }

    // Skip whites + ensure at least one white
    count = 0;
    for (; it != qrds_status.end(); it++) {
        if(!*it) {
            count++;
        }
        else if (count >= 1) {
            break;
        }
    }
    // Search for two blacks in a row
    count = 0;
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            count++;
            if (count == 1) {
                cond = true;
                break;
            }
        }
    }

    it = qrds_status.begin();
    bool val1 = *it;
    advance(it, 2);
    bool val2 = *it;

    it = qrds_status.end();
    advance(it, -1);
    bool val3 = *it;
    advance(it, -2);
    bool val4 = *it;

    bool cond2 = (val1 && val2) || (val3 && val4);

    return cond || cond2;

}

// T intersction required atleast 4 subsequent Black in a row
bool IntersectionManager::at_t_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);

    it = qrds_status.begin();

    bool cond = false;

    // Skip whites
    for (; it != qrds_status.end(); it++) {
        if(*it) {
            break;
        }
    }

    // Require four blacks in a row
    int count = 0;
    for (; it != qrds_status.end(); it++) {
        if(*it) {
            count++;
            if (count == 4) {
                cond = true;
                break;
            }
        }
        else {
            count = 0;
        }
    }
    return cond;

}

void IntersectionManager::handle_intersection() {
    switch (this->task) {
        case TASK_R: {
            Serial.println("Off ramp, going to home");

            if (this->side) {
                this->steer_left();
                this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            }
            else {
                this->steer_right();
                this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            }

            this->task = this->getNextTask();
            //increments to 0  = home position
            this->intersection_count = 0;
        }
            break;
        
        case TASK_S1: {
            
            Serial.println("Performing task medium post");

            switch(this->intersection_count) {

                case 0: {

                    if (this->side) {
                        Serial.println("Medium post: left turn at B");
                        this->steer_left();
                        this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
                    }
                    else {
                        Serial.println("Medium post: right turn at B");
                        this->steer_right();
                        this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
                    }

                    this->intersection_count++;
                    
                }
                    break;
                
                case 1: {
                    Serial.println("Grabbing medium stone");

                    this->motorsOff(300);

                    this->drive_system->update(-3.0, -3.0);
                    this->drive_system->actuate();
                    delay(200);
                    if (this->side) {
                        this->drive_system->update(-3.0, 0.93);
                    }
                    else {
                        this->drive_system->update(0.93, -3.0);
                    }
                    this->drive_system->actuate();
                    delay(100);
                    Serial.println("starting centering to medium post ");

                    while (true) {
                        
                        if (center_post(!(this->side))) {
                            break;
                        }
                        else {
                            // todo: readjust and centre on post
                        }

                    }

                    this->drive_system->update(0.86, 0.86);
                    this->drive_system->actuate();
                    delay(350);

                    this->wiggle(12,120);

                    this->motorsOff(300);

                    grabCrystal(1);
                    //openClaw(); // for now todo

                    this->motorsOff(300);

                    this->reverseAndTurn(600, 300, REVERSE_LEFT);
                    Serial.println("ending centering to medium post, going back to gauntlet ");
                    
                    this->intersection_count++;

                }
                    break;
                
                case 2: {

                    // Drive straight through B(Y)
                    this->intersection_count = 0;
                    this->tasksToDo.pop_back();
                    this->task = TASK_G1; // handoff to G1 task

                }
                    break;

            }

        }
            break;
        
        // case TASK_S2A: {

        //     Serial.println("Performing tall posts task");
        //     switch (this->intersection_count) {
        //         case 0: {
        //             Serial.println("Tall posts task: first Isect, slight right");
        //             //slight right at first Y intersection
        //             this->drive_system->update(0.94, 0.98);
        //             this->drive_system->actuate();
        //             delay(100);
        //             this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
        //         }
        //             break;
        //         case 1: {
        //             //turn right to find first post, wiggle, grab stone, U turn and keep going
        //             Serial.println("Tall posts task: first T, turn right");
        //             this->motorsOff(300);

        //             this->drive_system->update(-3.0, -3.0);
        //             this->drive_system->actuate();
        //             delay(200);
        //             this->drive_system->update(0.93, -3.0);
        //             this->drive_system->actuate();
        //             delay(100);
        //             Serial.println("starting centering to post #1 now ");

        //             center_post(true);

        //             this->drive_system->update(0.86, 0.86);
        //             this->drive_system->actuate();
        //             delay(350);

        //             this->wiggle(12,120);

        //             this->motorsOff(300);

        //             grabCrystal(0);
        //             //openClaw(); // for now todo

        //             this->motorsOff(300);

        //             this->reverseAndTurn(600, 300, REVERSE_RIGHT);
        //             Serial.println("ending centering to post #1 now, going back to recovery ");
        //             this->task = TASK_TO_RECOVERY;


        //         }
        //             break;

        //         case 2: {
        //             this->motorsOff(300);
        //             this->drive_system->update(-3.0, -3.0);
        //             this->drive_system->actuate();
        //             delay(200);
        //             this->drive_system->update(0.93, -3.0);
        //             this->drive_system->actuate();
        //             delay(100);
        //             Serial.println("starting centering to posts #2 now ");


        //             center_post(true);
        //             this->drive_system->update(0.86, 0.86);
        //             this->drive_system->actuate();
        //             delay(350);

        //             this->wiggle(12,120);

        //             this->motorsOff(300);

        //             grabCrystal(0);
                    
        //             this->reverseAndTurn(290, 500, REVERSE_LEFT);

        //             this->tasksToDo.pop_back();
        //             this->task = TASK_TO_RECOVERY;
        //             this->intersection_count = -1;
        //             Serial.println("gauntlet time ");

        //         }
        //             break;
        //     }
        // }
        //     break;
        // case TASK_TO_RECOVERY:
        // {
        //     Serial.println("Encountered intersection in recovery");
        //     isectType = readSerialIsectType();
        //     if (isectType == Y_ISECT) {
        //         Serial.println("detected y intersection ");

        //         // this->motorsOff(300);
                
        //         this->drive_system->update(0.94, 0.94);
        //         this->drive_system->actuate();
        //         delay(100);

        //         this->task = TASK_RECOVERY_TO_GAUNT_TO_HOME;
        //         this->intersection_count = -1;
        //     }
        //     else {
        //         Serial.println("Detected T intersection, coasting");
        //         this->motorsOff(300);
        //     }
        // }
        //     break;

        case TASK_G1:
        {
            switch (this->intersection_count) {
                case 0: {

                    Serial.println("Initiating gauntlet sequence...");

                    this->drive_system->update(0.86, 0.86);
                    this->drive_system->actuate();
                    
                    this->tape_sensor->update();
                    // ensure all (6) QRDs are off tape to begin turning back on
                    while (!(this->tape_sensor->is_far_left() || this->tape_sensor->is_far_right())) {
                        this->tape_sensor->update();
                    }

                    this->drive_system->update(0.0, 0.0);
                    this->drive_system->actuate();
                    delay(300);
                    
                    if (this->side) {
                        this->drive_system->update(-3.5, 0.0);
                    }
                    else {
                        this->drive_system->update(0.0, -3.5);
                    }
                    
                    this->drive_system->actuate();

                    this->tape_sensor->update();

                    // TODO: timeout?
                    long gauntlet_turn_time = millis();

                    //TODO add failsafe timeout if we dont reach this condition so we dont drive off course
                    if (this->side) {
                        while (!(this->tape_sensor->qrd2.is_on() && this->tape_sensor->qrd3.is_on())) {
                            this->tape_sensor->update();
                        }
                    }
                    else {
                        while (!(this->tape_sensor->qrd5.is_on() && this->tape_sensor->qrd4.is_on())) {
                            this->tape_sensor->update();
                        }
                    }

                    Serial.println("Gauntlet: followed back on tape");

                    this->drive_system->set_speed_add(-0.04);

                    this->intersection_count++;
                    handling_gauntlet = true;
                    gauntlet_timer = millis();

                }
                    break;
                
                case 1: {

                    // currently assuming CV sees the gauntlet

                    if (millis() - gauntlet_timer >= 1000) {
                        Serial.println("At gauntlet");
                        this->intersection_count++;
                        this->drive_system->set_speed_add(0.0);
                        this->wiggle(10, 150);
                        this->handle_gauntlet(2);
                        handling_gauntlet = false;
                        this->task = this->getNextTask();

                        // temp end program
                        delay(69420);
                    }

                }
                    break;
            
            }
        }
            break;

    } 
}

void IntersectionManager::initialize_tasksToDo() {
    this->tasksToDo.push_back(TASK_S1);
    //this->tasksToDo.push_back(TASK_S2A);
}

int IntersectionManager::getNextTask() {
    if (this->tasksToDo.empty()) {
        Serial.println("No more tasks!");
        while (true) {
            delay(1000);
            Serial.println("Spinning...");
        }
    }

    // Cycle task
    int retVal = this->tasksToDo.front();
    this->tasksToDo.pop_front();
    this->tasksToDo.push_back(retVal);

    return retVal;
}

void IntersectionManager::handle_gauntlet(int slot) {

    while (true) {

        if (this->place_stone(slot)) {
            return;
        }
        else {

            // TODO: incorporate failsafe with direction in mind

            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(300);

            this->drive_system->update(0.9, 0.9);
            this->drive_system->actuate();
            delay(500);

            wiggle(10, 150);

            //this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);

        }

    }

}

bool IntersectionManager::place_stone(int slot) {

    // TODO: better algo could be to move forward y until close enough
    //  as y is moving, whenever x deviates too much, readjust x and then go forward in y

    // 1. Minimize x

    // TODO: use flag to set init state

    // initial values should fail the while loop checks

    int x = slot <= 2 ? -999 : 999;
    int y = 69420;

    this->drive_system->update(0.0, 0.0);
    this->drive_system->actuate();

    Serial.println("Initiating deposit sequence...");

    bool complete = false;
    long timeout = millis();

    do {

        // Right now: turn left wheel back to hole 0

        // May be useful to find post when lost

        // ~20-25px per cm (crude estimate, height dependent, etc.)
        // currently have 0.05% pwm per px

        // turn left
        if (x < 0) {
            this->drive_system->update(-3.1, 0.86);
        }
        else if (x > 0) {
            this->drive_system->update(0.86, -3.1);
        }
        this->drive_system->actuate();

        // TODO: tune values

        for (int i = 0; i < 90; i++) {

            if (Serial.read() == 'G') {

                // Go through the slots before it
                for (int i = 0; i < slot; i++) {
                    Serial.readStringUntil(',');
                    Serial.readStringUntil(';');
                }

                x = Serial.readStringUntil(',').toInt();
                y = Serial.readStringUntil(';').toInt();

                if (fabs(x) <= 12) {
                    complete = true;
                    break;
                }

            }

            delay(1);

        }

        if (complete) {
            break;
        }

        // Pause motors
        if (x < 0) {
            this->drive_system->update(0.0, 0.7);
        }
        else if (x > 0) {
            this->drive_system->update(0.7, 0.0);
        }
        this->drive_system->actuate();

        for (int i = 0; i < 70; i++) {

            if (Serial.read() == 'G') {

                // Go through the slots before it
                for (int i = 0; i < slot; i++) {
                    Serial.readStringUntil(',');
                    Serial.readStringUntil(';');
                }

                x = Serial.readStringUntil(',').toInt();
                y = Serial.readStringUntil(';').toInt();

                if (fabs(x) <= 12) {
                    complete = true;
                    break;
                }

            }

            delay(1);

        }

        if (millis() - timeout > 15000) {
            Serial.println("gauntlet timed out");
            return false;
        }

    } while (fabs(x) > 12);

    this->drive_system->update(0.0, 0.0);
    this->drive_system->actuate();

    // Serial.println("OMFG");

    // 3. Deposit stone
    // todo: match deposit crystal number to circle
    // closeClaw(50);

    int znum = -1;

    switch (slot) {
        case 0:
        case 5:
            znum = 2;
            break;
        case 1:
        case 4:
            znum = 1;
            break;
        case 2:
        case 3:
            znum = 0;
            break;
    }

    depositCrystal(znum, true);

    return true;

}

// true turns the robot cw initially, ccw for false
// TODO add debounce and check counter clowckwise turn
bool IntersectionManager::center_post(bool init_dir) {

    // 1. Minimie x

    // initial values should fail the while loop checks
    // todo: set flag for init
    int x = init_dir ? 999 : -999;
    int y = 69420;

    Serial.println("Initiating post-centering sequence...");

    bool complete = false;

    long timeout = millis();

    do {

        // Right now: turn left wheel back to hole 0

        // May be useful to find post when lost

        // ~20-25px per cm (crude estimate, height dependent, etc.)
        // currently have 0.05% pwm per px

        // turn left
        if (x < 0) {
            this->drive_system->update(-3.1, 0.86);
        }
        else if (x > 0) {
            this->drive_system->update(0.86, -3.1);
        }
        this->drive_system->actuate();

        // TODO: tune values

        for (int i = 0; i < 90; i++) {

            if (Serial.read() == 'P') {

                x = Serial.readStringUntil(',').toInt();
                y = Serial.readStringUntil(';').toInt();

                if (fabs(x) <= 90) {
                    complete = true;
                    break;
                }

            }

            delay(1);

        }

        if (complete) {
            break;
        }

        // Pause motors
        if (x < 0) {
            this->drive_system->update(0.0, 0.7);
        }
        else if (x > 0) {
            this->drive_system->update(0.7, 0.0);
        }
        this->drive_system->actuate();

        for (int i = 0; i < 40; i++) {

            if (Serial.read() == 'P') {

                x = Serial.readStringUntil(',').toInt();
                y = Serial.readStringUntil(';').toInt();

                if (fabs(x) <= 90) {
                    complete = true;
                    break;
                }

            }

            delay(1);

        }

        if (millis() - timeout > 8000) {
            Serial.println("post timed out");
            return false;
        }

    } while (fabs(x) > 90);

    Serial.println("Centred on post");

    this->drive_system->update(0.0, 0.0);
    this->drive_system->actuate();

    return true;

}

void IntersectionManager::steer_left() {

    QrdSensor* qrds[8] = {&this->tape_sensor->qrd0, &this->tape_sensor->qrd1,
                          &this->tape_sensor->qrd2, &this->tape_sensor->qrd3,
                          &this->tape_sensor->qrd4, &this->tape_sensor->qrd5,
                          &this->tape_sensor->qrd6, &this->tape_sensor->qrd7};

    this->drive_system->update(-2.7, -2.7);
    this->drive_system->actuate();
    while (!this->at_y_intersection_lenient()) {
        this->tape_sensor->update();
    }

    int qrd_idx = this->first_black_sensor();

    Serial.println("steering left");

    this->drive_system->update(-2.7, 0.93);
    this->drive_system->actuate();

    long timeout = millis();
    // Do until qrd5 is on tape
    while ((qrd_idx <= 4) || (millis() - timeout <= 600)) {
        // TODO: maybe set far off state
        this->tape_sensor->update();

        if ((*qrds[qrd_idx]).is_on()) {
            qrd_idx++;
        }

    }

}

void IntersectionManager::steer_right() {

    // Sneak: just make the array backwards compared to steer_left
    QrdSensor* qrds[8] = {&this->tape_sensor->qrd7, &this->tape_sensor->qrd6,
                          &this->tape_sensor->qrd5, &this->tape_sensor->qrd4,
                          &this->tape_sensor->qrd3, &this->tape_sensor->qrd2,
                          &this->tape_sensor->qrd1, &this->tape_sensor->qrd0};

    this->drive_system->update(-2.7, -2.7);
    this->drive_system->actuate();
    while (!this->at_y_intersection_lenient()) {
        this->tape_sensor->update();
    }

    Serial.println("steering right");

    int qrd_idx = 7 - this->last_black_sensor();

    this->drive_system->update(0.93, -2.7);
    this->drive_system->actuate();

    long timeout = millis();
    // Do until qrd1 is on tape
    while ((qrd_idx <= 4) || (millis() - timeout <= 600)) {
        // TODO: maybe set far off state
        this->tape_sensor->update();

        if ((*qrds[qrd_idx]).is_on()) {
            qrd_idx++;
        }

    }

}

int IntersectionManager::first_black_sensor() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    vector<bool>::iterator it = qrds_status.begin();

    // Goal: find index of first white in white island
    int idx = 0;

    // Get to a black
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            idx++;
            return idx;
        }
        idx++;
    }

    // default return 0
    return 0;

}

int IntersectionManager::last_black_sensor() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();
    reverse(qrds_status.begin(), qrds_status.end());

    vector<bool>::iterator it = qrds_status.begin();

    // Goal: find index of last white in white island
    int idx = 0;

    // Get to a black
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            idx++;
            return 7 - idx;
        }
        idx++;
    }

    // default return 7
    return 7;

}

void IntersectionManager::test_stone(int n) {
    place_stone(n);
}
