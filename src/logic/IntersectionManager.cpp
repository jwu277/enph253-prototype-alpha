#include <Arduino.h>

#include "sensors/MainTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include "logic/IntersectionManager.hpp"

#define TURN_COUNTER_MAX 100
#define DELAY_TIME 800

double last_intersection_time=0;

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* tape_sensor,
    DriveSystem* drive_system) {
        
    // Modules
    this->tape_sensor = tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

}

void IntersectionManager::update() {

    // todo: organize into function
    // unignore
    if (fabs(*this->tape_sensor->get_x_ptr()) <= 0.2) {
        this->tape_sensor->init_sensor_weights();
    }

    // TEMP: for now
    if (this->at_t_intersection()||this->at_y_intersection()) {

        if (this->intersection_count % 2 == 0){
            pwm_start(PB_4, 1000000, 10, 10, 0);
        }
        else {
            pwm_start(PB_4, 1000000, 10, 0, 0);
        }
        pwm_start(PA_8, 1000000, 10, 10, 0);

        this->handle_intersection();
        double new_time = millis();
        if(last_intersection_time- new_time<DELAY_TIME)
            this->intersection_count++;
            last_intersection_time = new_time;
            // Serial.println(last_intersection_time-new_time);


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
    qrds_status.insert(qrds_status.end(), *qrds_status.end());
    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    qrds_status.insert(qrds_status.end(), *qrds_status.end());

    bool cond = false;

    vector<bool>::iterator it = qrds_status.begin();

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

    return cond;

}

bool IntersectionManager::at_t_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    vector<bool>::iterator it = qrds_status.begin();

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
            this->drive_system->update(0.95, 0.80);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(500);
            // this->drive_system->update(0.85, -2.3);
            // this->drive_system->actuate();
            // delay(800);
            // this->drive_system->update(0.95, 0.95);
            // this->drive_system->actuate();
            // delay(200);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(50000000);
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
            this->drive_system->update(0.8, 0.8);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);

            // this->tape_sensor->ignore_right_sensors();
            break;
        case 2:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(250000);
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
