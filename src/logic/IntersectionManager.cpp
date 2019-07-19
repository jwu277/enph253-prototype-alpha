#include <Arduino.h>

#include "sensors/MainTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include "logic/IntersectionManager.hpp"

#include "claw_system.h"

#define TURN_COUNTER_MAX 100
#define DELAY_TIME 200

long last_intersection_time = millis();

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* tape_sensor,
    DriveSystem* drive_system) {
        
    // Modules
    this->tape_sensor = tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

}

//TODO make function to wiggle left and right to get clean lock on
void wiggle() {


}

void IntersectionManager::update() {

    // todo: organize into function
    // unignore
    if (fabs(*this->tape_sensor->get_x_ptr()) <= 0.2) {
        this->tape_sensor->init_sensor_weights();
    }

    // TEMP: for now
    if (this->at_t_intersection()||this->at_y_intersection()) {
        
        // if (this->intersection_count % 2 == 0){
        //     pwm_start(PB_4, 1000000, 10, 10, 0);
        // }
        // else {
        //     pwm_start(PB_4, 1000000, 10, 0, 0);
        // }
        // pwm_start(PA_8, 1000000, 10, 10, 0);

        //
        long new_time = millis();
        if(new_time - last_intersection_time >= DELAY_TIME) {
            this->handle_intersection();
            this->intersection_count++;   
            last_intersection_time = new_time;
            // Serial.println(last_intersection_time-new_time);
        }


    }
    else {
        pwm_start(PA_8, 1000000, 10, 0, 0);
    }
    /*
    if (this->at_t_intersection()) {

        // pwm_start(PB_4, 1000000, 10, 10, 0);
        pwm_start(PA_8, 1000000, 10, 10, 0);

        //this->handle_intersection();

        //this->intersection_count++;

    }*/

}

bool IntersectionManager::at_y_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    // duplicate end elements
    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);
    // qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    // qrds_status.insert(qrds_status.end(), *qrds_status.end());

    bool cond = false;

    it = qrds_status.begin();

    // Search for two blacks
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

    /*
    it = qrds_status.begin();
    for (; it != qrds_status.end(); it++) {
        Serial.print(*it ? "1" : "0");
        Serial.print("     ");
    }
    Serial.println();*/

    bool cond2 = (val1 && val2) || (val3 && val4);

    return cond || cond2;

}

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

    //this->tape_sensor->ignore_right_sensors();
    //this->tape_sensor->ignore_left_sensors();

    switch (this->intersection_count) {
        case 0:
            // this->drive_system->update(0.95, 0.80);
            // this->drive_system->actuate();
            // delay(400);
            // this->drive_system->update(0.0, 0.0);
            // this->drive_system->actuate();
            // delay(500);
            // // this->drive_system->update(0.85, -2.3);
            // // this->drive_system->actuate();
            // // delay(800);
            // // this->drive_system->update(0.95, 0.95);
            // // this->drive_system->actuate();
            // // delay(200);
            // this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            // this->drive_system->update(0.0, 0.0);
            // this->drive_system->actuate();
            // delay(50000000);
            // this->drive_system->update(0.8, 0.8);
            // this->drive_system->actuate();
            // delay(300);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.85, 0.5);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(0.85, 0.85);
            this->drive_system->actuate();
            delay(200);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            break;
        // case 1:
            // cheap disable/debounce
            // this->drive_system->update(0.85, 0.85);
            // this->drive_system->actuate();
            // delay(200);
           
        /*
        case 1:
            this->drive_system->update(0.85, -1.8);
            this->drive_system->actuate();
            delay(700);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            break;
        */
        case 1:
            this->drive_system->update(0.7, 0.85);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);

            // this->tape_sensor->ignore_right_sensors();
            break;
        case 2:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(500);
            this->drive_system->update(-2.8, -2.8);
            this->drive_system->actuate();
            delay(400);
            // this->tape_sensor->update();
            // while (!this->at_t_intersection()) {
            //     this->tape_sensor->update();
            //     delay(1);
            // }
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(500);
            this->drive_system->update(-2.8, 0.70);
            this->drive_system->actuate();
            delay(350);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-2.8, -0.1);
            this->drive_system->actuate();
            delay(650);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(0.80, 0.80);
            this->drive_system->actuate();
            delay(800);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            // wiggle();

            //wiggle here
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            // end wiggle

            delay(2000); //TODO get rock here
            grabCrystal();

            this->drive_system->update(-2.8, -2.8);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(-2.8, -0.1);
            this->drive_system->actuate();
            delay(400);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            break;
        //case 4:
            // this->drive_system->update(0.7, 0.85);
            // this->drive_system->actuate();
            // delay(300);
            // this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
        case 3:
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(500);
            this->drive_system->update(0.85, 0.85);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(0.85, -2.8);
            this->drive_system->actuate();
            delay(900);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            // this->drive_system->update(-0.1, -2.8);
            // this->drive_system->actuate();
            // delay(900);
            // this->drive_system->update(-0.1, -0.1);
            // this->drive_system->actuate();
            // delay(300);
            this->drive_system->update(0.85, 0.85);
            this->drive_system->actuate();
            delay(900);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(600);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(0.80, -0.1);
            this->drive_system->actuate();
            delay(50);
            this->drive_system->update(-0.1, 0.80);
            this->drive_system->actuate();
            delay(50);
            delay(2000); //TODO drop rock here
            break;
            
    }

    //right turn 
    // if (this->at_y_intersection()){
    //         drive_system->update(.85, -2.7);
    //         drive_system->actuate();
    //         delay(50);
    // }
}
// void IntersectionManager::handle_intersection() {

//     switch(this->intersection_count) {

//         case 0:

//             drive_system->update(1.0, 1.0);
//             drive_system->actuate();
//             delay(500);

//             drive_system->turn_left();
//             drive_system->actuate();
//             delay(400);

//             tape_sensor->set_state(MainTapeSensor::RIGHT);

//             break;

//         case 1:
            
//             drive_system->update(0.0, 0.0);
//             drive_system->actuate();
//             delay(4000);

//             break;

//     }

// }


/*
void IntersectionManager::update() {

    if (is_turning) {
        
        this->drive_system->turn_left();

    }
    else {

        bool at_intersection = this->main_tape_sensor->is_both_on() &&
            (this->side_tape_sensor->is_left_on() || this->side_tape_sensor->is_right_on());

        if (at_intersection) {
            this->is_turning = true;
            this->turn_counter = 0;
        }

    }
    
}

void IntersectionManager::increment_turn_counter() {

    this->turn_counter++;

    if (this->turn_counter >= TURN_COUNTER_MAX) {
        this->is_turning = false;
    }

}
*/
