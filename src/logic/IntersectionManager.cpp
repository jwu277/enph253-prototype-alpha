#include <Arduino.h>

#include "logic/IntersectionManager.hpp"

#define TURN_COUNTER_MAX 100

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* main_tape_sensor,
    SideTapeSensor* side_tape_sensor, DriveSystem* drive_system) {
        
    // Modules
    this->main_tape_sensor = main_tape_sensor;
    this->side_tape_sensor = side_tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

}

void IntersectionManager::update() {

    bool at_intersection = this->main_tape_sensor->is_both_on() &&
            (this->side_tape_sensor->is_left_on() || this->side_tape_sensor->is_right_on());

    // TEMP: for now
    if (at_intersection) {

        this->handle_intersection();

        this->intersection_count++;

    }

}

void IntersectionManager::handle_intersection() {

    switch(this->intersection_count) {

        case 0:

            drive_system->drive_forward();
            drive_system->actuate();
            delay(400);

            drive_system->turn_left();
            drive_system->actuate();
            delay(400);

            break;

        case 1:
            
            drive_system->update(0.0, 0.0);
            drive_system->actuate();
            delay(4000);

            break;

    }

}

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
