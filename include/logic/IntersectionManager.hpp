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

        //Frequently used movement functions
        void wiggle(int numOfWiggles, int wiggleHalfPeriod);
        void reverseAndTurn(int reverseTime, int turnTime, bool dir);
        void motorsOff(int duration);

        // TODO: different intersection handlers for Y vs. T junctions
        // Intersection Handler
        void handle_intersection();
        int task;

        bool readSerialIsectType();
        bool isectType;

        // temp function?
        void handle_gauntlet();

        // Returns success
        bool place_stone(int slot);

    public:

        // Constructor
        IntersectionManager(MainTapeSensor* main_tape_sensor, DriveSystem* drive_system);

        // Update logic in SW
        void update();


        // Increment turn counter
        //void increment_turn_counter();

};

#endif
