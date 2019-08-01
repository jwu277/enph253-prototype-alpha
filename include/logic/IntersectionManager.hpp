#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

#define Y_ISECT true
#define T_ISECT false
#define ISECT_VOTES 3

using namespace std;

#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include <deque> 


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

        //intersection voting for classification
        deque<bool> isectTypeVotes;
        bool isectType;

        // Intersection Detectors
        bool at_y_intersection();
        bool at_t_intersection();

        bool at_y_intersection_lenient();

        bool at_intersection();
        bool at_intersection_and_get_vote();
        bool classify_intersection();

        // TODO: different intersection handlers for Y vs. T junctions
        // Intersection Handler
        void handle_intersection();

        // temp function?
        void handle_gauntlet();

        void handle_post();

        void get_to_post();

        void steer_left();
        void steer_right();

        int first_black_sensor();
        int last_black_sensor();

    public:

        // Constructor
        IntersectionManager(MainTapeSensor* main_tape_sensor, DriveSystem* drive_system);

        // Update logic in SW
        void update();


        // Increment turn counter
        //void increment_turn_counter();

};

#endif
