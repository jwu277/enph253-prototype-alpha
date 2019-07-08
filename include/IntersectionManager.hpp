#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

#include "MainTapeSensor.hpp"
#include "SideTapeSensor.hpp"
#include "DriveSystem.hpp"


// IntersectionManager is a logic controller for handling intersections
class IntersectionManager {

    private:

        // Modules
        MainTapeSensor* main_tape_sensor;
        SideTapeSensor* side_tape_sensor;
        DriveSystem* drive_system;

    public:

        // Constructor
        IntersectionManager(MainTapeSensor* main_tape_sensor,
            SideTapeSensor* side_tape_sensor, DriveSystem* drive_system);

        // Update logic in SW
        void update();

};

#endif
