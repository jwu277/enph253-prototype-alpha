#include <Arduino.h>

#include "sensors/MainTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include "logic/IntersectionManager.hpp"

#include "claw_system.h"

#define TURN_COUNTER_MAX 100
#define DELAY_TIME 200

unsigned long last_intersection_time = millis();

unsigned long gauntlet_timer;

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* tape_sensor,
    DriveSystem* drive_system) {
        
    // Modules
    this->tape_sensor = tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

    this->gauntlet_state = 0;

}

//TODO make function to wiggle left and right to get clean lock on
void wiggle() {


}

void IntersectionManager::update() {

    // temp: handle gauntlet
    // TODO: manage intersection increments
    if (this->intersection_count == 7) {
        this->handle_gauntlet();
    } 

    // TEMP: for now
    // else if (this->at_t_intersection()||this->at_y_intersection()) {
    else if (this->at_intersection()) {

        unsigned long new_time = millis();
        //Debounce in case intersection triggered by accident due to oscilation 
        if(new_time - last_intersection_time >= DELAY_TIME) {
            // this->handle_intersection();
            this->steer_left();
            this->intersection_count++;
            last_intersection_time = new_time;
        }
    }
    else {
        this->tape_sensor->init_sensor_weights();
    }

}

bool IntersectionManager::at_intersection() {
    return this->at_y_intersection() || this->at_t_intersection();
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
            if (count == 2) {
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
            if (count == 2) {
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

    switch (this->intersection_count) {
        case 0:
            this->steer_left();
            // TODO: figure out far off state (for all junctions)
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            break;
        case 1:
            this->steer_left();
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            break;
        case 2:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.93, -3.0);
            this->drive_system->actuate();
            delay(280);
            this->drive_system->update(0.86, 0.86);
            this->drive_system->actuate();
            delay(350);

            //WIGGLE
            for (int i = 0; i < 12; i++) {
                this->drive_system->update(0.93, -0.1);
                this->drive_system->actuate();
                delay(120);
                this->drive_system->update(-0.1, 0.93);
                this->drive_system->actuate();
                delay(120);
            }

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            grabCrystal();
            openClaw();

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-3.0, .98);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            break;
        case 3:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.93, -3.0);
            this->drive_system->actuate();
            delay(280);
            this->drive_system->update(0.86, 0.86);
            this->drive_system->actuate();
            delay(350);

            // WIGGLE
            for (int i = 0; i < 12; i++) {
                this->drive_system->update(0.93, -0.1);
                this->drive_system->actuate();
                delay(120);
                this->drive_system->update(-0.1, 0.93);
                this->drive_system->actuate();
                delay(120);
            }

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            grabCrystal();
            
            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(290);
            this->drive_system->update(.98, -3.0);
            this->drive_system->actuate();
            delay(500);
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            break;
            
        case 4:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);
            break;
        
        case 5:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);
            break;

        // Gauntlet here
        case 6:
            break;
    }
}
void IntersectionManager::handle_gauntlet() {

    switch(this->gauntlet_state) {

        case 0:
            
            this->drive_system->update(-1.0, -1.0);
            this->drive_system->actuate();
            delay(550);
            
            this->drive_system->update(-3.5, 0.0);
            this->drive_system->actuate();

            this->tape_sensor->update();

            //TODO add failsafe timeout if we dont reach this condition so we dont drive off course
            while (!(this->tape_sensor->qrd2.is_on() || this->tape_sensor->qrd3.is_on())) {
                this->tape_sensor->update();
            }
            gauntlet_timer = millis();

            this->drive_system->set_speed_add(-0.04);

            this->gauntlet_state++;

            break;

        case 1:
            // keep pid going for ____ milliseconds and then increment to next state
            if (millis() - gauntlet_timer >= 1600) {

                this->gauntlet_state++;

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(200);

                this->drive_system->update(-3.0, -3.0);
                this->drive_system->actuate();
                delay(500);

                this->drive_system->set_speed_add(0.0);

                this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);

                gauntlet_timer = millis();

            }

            break;
        case 2:
            
            // keep pid going for ____ milliseconds and then increment to next state
            if (millis() - gauntlet_timer >= 1600) {

                this->drive_system->set_speed_add(0.0);

                this->gauntlet_state++;

                for (int i = 0; i < 16; i++) {
                    this->drive_system->update(0.93, -0.1);
                    this->drive_system->actuate();
                    delay(120);
                    this->drive_system->update(-0.1, 0.93);
                    this->drive_system->actuate();
                    delay(120);
                }

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(500);

                this->drive_system->update(-3.5, 0.88);
                this->drive_system->actuate();
                delay(280);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(400);

                closeClaw();

                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(false);
                digitalWrite(STEPPERENABLE, HIGH);
                delay(1000);
                openClaw();
                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(true);
                moveZToExtreme(HOME);
                digitalWrite(STEPPERENABLE, HIGH);

                this->drive_system->update(0.94, 0.80);
                this->drive_system->actuate();
                delay(500);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(200);

                this->drive_system->update(0.88, -3.5);
                this->drive_system->actuate();
                delay(280);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(400);

                closeClaw();

                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(false);
                digitalWrite(STEPPERDIR, DOWN);
                for(int i = 0; i < 100; i++) {
                    stepperPulse();
                }
                digitalWrite(STEPPERENABLE, HIGH);

                delay(1000);
                openClaw();
                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(true);
                moveZToExtreme(HOME);
                digitalWrite(STEPPERENABLE, HIGH);

                delay(69420);
            }
            break;
    }
}

void IntersectionManager::get_to_post() {

    // 1. Get post in view
    this->drive_system->update(0.88, 0.0); // spin right
    this->drive_system->actuate();
    while (!Serial.available());

    // 2. Navigate to post
    // TODO: complete
    // Current strategy: proportional minimize x and 
    int x;
    int y;

    do {

        // May be useful to find post when lost

        // TODO: incorporate camera offset

        // ~20-25px per cm (crude estimate, height dependent, etc.)
        // currently have 0.05% pwm per px

        if (Serial.read() == 'P') {

            x = Serial.readStringUntil(',').toInt();
            y = Serial.readStringUntil(';').toInt();

            if (x > 0) {
                // Turn right
                this->drive_system->update(0.8 + 0.0005 * x, -(0.8 + 0.0005 * x));
            }
            else if (x < 0) {
                // Turn left
                this->drive_system->update(-(0.8 - 0.0005 * x), 0.8 - 0.0005 * x);
            }

        }

        this->drive_system->actuate();

    } while (fabs(x) > 30);

    // Go forward into post
    // TODO: use y feedback, and maybe swim
    this->drive_system->update(0.87, 0.87);
    this->drive_system->actuate();
    delay(800);

}

void IntersectionManager::handle_post() {
    get_to_post();
    grabCrystal();
}

void IntersectionManager::steer_left() {

    QrdSensor* qrds[8] = {&this->tape_sensor->qrd0, &this->tape_sensor->qrd1,
                          &this->tape_sensor->qrd2, &this->tape_sensor->qrd3,
                          &this->tape_sensor->qrd4, &this->tape_sensor->qrd5,
                          &this->tape_sensor->qrd6, &this->tape_sensor->qrd7};

    int qrd_idx = 4;

    this->drive_system->update(-2.7, 0.93);
    this->drive_system->actuate();

    long timeout = millis();
    // Do until qrd6 is on tape
    while (qrd_idx <= 6) {
        // TODO: maybe set far off state
        this->tape_sensor->update();

        if ((*qrds[qrd_idx]).is_on()) {
            qrd_idx++;
        }

        if (millis() - timeout >= 2400) {
            break;
        }

    }

}

void IntersectionManager::steer_right() {

    this->drive_system->update(0.86, 0);
    this->drive_system->actuate();

    long timeout = millis();
    while (!(this->tape_sensor->qrd0.is_on() && !this->at_intersection())) {
        // TODO: maybe set far off state
        this->tape_sensor->update();
        if (millis() - timeout >= 2400) {
            break;
        }
    }

}

int IntersectionManager::first_black_sensor() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    vector<bool>::iterator it = qrds_status.begin();

    // Goal: find index of first white
    int idx = 0;

    // Get to a black
    for (; it != qrds_status.end(); it++) {
        if (*it) {
            idx++;
            return idx;
        }
        idx++;
    }

    // default return 1
    return 4;

}
