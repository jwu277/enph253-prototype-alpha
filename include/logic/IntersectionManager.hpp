#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

using namespace std;

#define TASK_RAMP_TO_HOME 0
#define TASK_RECOVERY_TO_GAUNT_TO_HOME 1
#define TASK_TO_RECOVERY 3
#define TASK_TALL_POSTS 4
#define TASK_SINGLE_POST 5
#define TASK_SHORT_POSTS 6
#define TASK_GAUNT_TO_HOME 10
#define TASK_HOME_TO_GAUNT 11

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

        // Intersection Detectors
        bool at_y_intersection();
        bool at_t_intersection();
        bool at_y_intersection_lenient();
        

        //Frequently used movement functions
        void wiggle(int numOfWiggles, int wiggleHalfPeriod);
        void reverseAndTurn(int reverseTime, int turnTime, bool dir);
        void motorsOff(int duration);

        // TODO: different intersection handlers for Y vs. T junctions
        // Intersection Handler
        void handle_intersection();
        int task;
        int getNextTask();
        deque <int> tasksToDo;
        

        bool readSerialIsectType();
        bool isectType;

        // temp function?
        void handle_gauntlet();

        bool place_stone(int slot);
        void center_post(bool dir);

        void uturn(bool dir);
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
