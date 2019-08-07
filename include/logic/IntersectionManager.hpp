#ifndef INTERSECTIONMANAGER
#define INTERSECTIONMANAGER

using namespace std;

// Remember baton analogy

#define TASK_R 0 // ramp to home

#define TASK_S1 11 // 1 medium post, Tier 1/2/3
#define TASK_S2A 12 // 1 tall post, Tier 2
#define TASK_S2B 13 // 2 tall posts, Tier 3

#define TASK_G1 21 // gauntlet task for S1
#define TASK_G2A 22 // gauntlet task for S2
#define TASK_G2B 23 // gauntlet task for S2,3

#include "sensors/MainTapeSensor.hpp"
#include "sensors/SideTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include <deque>

// IntersectionManager is a logic controller for handling intersections
class IntersectionManager {

    private:

        bool side;

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
        void initialize_tasksToDo();
        int getNextTask();
        deque <int> tasksToDo;
        

        bool readSerialIsectType();
        bool isectType;

        // temp function?
        void handle_gauntlet(int slot, bool inClaw);

        bool place_stone(int slot, bool inClaw);
        bool center_post(bool);

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

        void set_side(bool);

        // Increment turn counter
        //void increment_turn_counter();
        
        void test_stone(int);

};

#endif
