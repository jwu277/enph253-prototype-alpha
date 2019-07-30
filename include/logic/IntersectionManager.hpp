#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"


// IntersectionManager is a logic controller for handling intersections
class IntersectionManager {

    private:

        // Modules
        MainTapeSensor* tape_sensor;
        DriveSystem* drive_system;

        // State
        // TODO: update
        int intersection_count;

        // temp gauntlet state
        int gauntlet_state;

        // Intersection Detectors
        bool at_y_intersection();
        bool at_t_intersection();

        // TODO: different intersection handlers for Y vs. T junctions
        // Intersection Handler
        void handle_intersection();

        // temp function?
        void handle_gauntlet();

        void handle_post();

        void get_to_post();

        void steer_left();

    public:

        // Constructor
        IntersectionManager(MainTapeSensor* main_tape_sensor, DriveSystem* drive_system);

        // Update logic in SW
        void update();


        // Increment turn counter
        //void increment_turn_counter();

};

#endif
