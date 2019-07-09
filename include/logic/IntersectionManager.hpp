#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"


// IntersectionManager is a logic controller for handling intersections
class IntersectionManager {

    private:

        // Modules
        MainTapeSensor* main_tape_sensor;
        SideTapeSensor* side_tape_sensor;
        DriveSystem* drive_system;

        // State
        // TODO: update
        int intersection_count;

        // Intersection Handler
        void handle_intersection();

    public:

        // Constructor
        IntersectionManager(MainTapeSensor* main_tape_sensor,
            SideTapeSensor* side_tape_sensor, DriveSystem* drive_system);

        // Update logic in SW
        void update();

        // Increment turn counter
        //void increment_turn_counter();

};

#endif
