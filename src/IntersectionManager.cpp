#include <Arduino.h>

#include "IntersectionManager.hpp"

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* main_tape_sensor,
    SideTapeSensor* side_tape_sensor, DriveSystem* drive_system) {
        
    // Modules
    this->main_tape_sensor = main_tape_sensor;
    this->side_tape_sensor = side_tape_sensor;
    this->drive_system = drive_system;

}

void IntersectionManager::update() {

    bool at_intersection = this->main_tape_sensor->is_both_on() &&
        (this->side_tape_sensor->is_left_on() || this->side_tape_sensor->is_right_on());

    if (at_intersection) {
        this->drive_system->turn_left();
    }
    
}
