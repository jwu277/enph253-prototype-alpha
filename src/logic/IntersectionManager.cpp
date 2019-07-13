#include <Arduino.h>

#include "logic/IntersectionManager.hpp"

#define TURN_COUNTER_MAX 100

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
    if (fabs(*this->tape_sensor->get_x_ptr()) >= 0.2) {
        this->tape_sensor->init_sensor_weights();
    }

    // TEMP: for now
    if (this->at_y_intersection()) {

        pwm_start(PB_4, 1000000, 10, 10, 0);

        this->handle_intersection();

        //this->intersection_count++;

    }

}

bool IntersectionManager::at_y_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    // duplicate end elements
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

    // Skip whites
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

void IntersectionManager::handle_intersection() {

    this->tape_sensor->ignore_right_sensors();

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
