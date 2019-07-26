#include <Arduino.h>

#include "sensors/MainTapeSensor.hpp"
#include "actuators/DriveSystem.hpp"
#include "logic/IntersectionManager.hpp"

#include "claw_system.h"

#define TURN_COUNTER_MAX 100
#define DELAY_TIME 200

unsigned long last_intersection_time = millis();

unsigned long gauntlet_timer;

// Constructor
IntersectionManager::IntersectionManager(MainTapeSensor* tape_sensor,
    DriveSystem* drive_system) {
        
    // Modules
    this->tape_sensor = tape_sensor;
    this->drive_system = drive_system;

    // State
    this->intersection_count = 0;

    this->gauntlet_state = 0;

}

//TODO make function to wiggle left and right to get clean lock on
void wiggle() {


}

void IntersectionManager::update() {

    // temp: handle gauntlet
    // TODO: manage intersection increments
    if (this->intersection_count == 7) {
        this->handle_gauntlet();
    } 

    // TEMP: for now
    else if (this->at_t_intersection()||this->at_y_intersection()) {

        //Serial.print("YASSS");
        
        // if (this->intersection_count % 2 == 0){
        //     pwm_start(PB_4, 1000000, 10, 10, 0);
        // }
        // else {
        //     pwm_start(PB_4, 1000000, 10, 0, 0);
        // }
        // pwm_start(PA_8, 1000000, 10, 10, 0);

        //
        // Serial.println("INTERSECTION!");
        unsigned long new_time = millis();
        if(new_time - last_intersection_time >= DELAY_TIME) {
            this->handle_intersection();
            this->intersection_count++;
            last_intersection_time = new_time;
            // Serial.println(last_intersection_time-new_time);
        }


    }
    // todo: organize into function
    // unignore
    /*
    if (fabs(*this->tape_sensor->get_x_ptr()) <= 0.2) {
        this->tape_sensor->init_sensor_weights();
    }*/
    /*
    if (this->at_t_intersection()) {

        // pwm_start(PB_4, 1000000, 10, 10, 0);
        pwm_start(PA_8, 1000000, 10, 10, 0);

        //this->handle_intersection();

        //this->intersection_count++;

    }*/

}

bool IntersectionManager::at_y_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    // duplicate end elements
    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);
    // qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    // qrds_status.insert(qrds_status.end(), *qrds_status.end());

    bool cond = false;

    it = qrds_status.begin();

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

    // Skip whites + ensure at least one white
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

    it = qrds_status.begin();
    bool val1 = *it;
    advance(it, 2);
    bool val2 = *it;

    it = qrds_status.end();
    bool val3 = *it;
    advance(it, -2);
    bool val4 = *it;

    /*
    it = qrds_status.begin();
    for (; it != qrds_status.end(); it++) {
        Serial.print(*it ? "1" : "0");
        Serial.print("     ");
    }
    Serial.println();*/

    bool cond2 = (val1 && val2) || (val3 && val4);

    return cond || cond2;

}

bool IntersectionManager::at_t_intersection() {
    
    vector<bool> qrds_status = this->tape_sensor->get_qrds_status();

    qrds_status.insert(qrds_status.begin(), *qrds_status.begin());
    vector<bool>::iterator it = qrds_status.end();
    advance(it, -1);
    qrds_status.push_back(*it);

    it = qrds_status.begin();

    bool cond = false;

    // Skip whites
    for (; it != qrds_status.end(); it++) {
        if(*it) {
            break;
        }
    }

    // Require four blacks in a row
    int count = 0;
    for (; it != qrds_status.end(); it++) {
        if(*it) {
            count++;
            if (count == 4) {
                cond = true;
                break;
            }
        }
        else {
            count = 0;
        }
    }

    return cond;

}

void IntersectionManager::handle_intersection() {

    //this->tape_sensor->ignore_right_sensors();
    //this->tape_sensor->ignore_left_sensors();

    switch (this->intersection_count) {
        case 0:
            // this->drive_system->update(0.95, 0.80);
            // this->drive_system->actuate();
            // delay(400);
            // this->drive_system->update(0.0, 0.0);
            // this->drive_system->actuate();
            // delay(500);
            // // this->drive_system->update(0.85, -2.3);
            // // this->drive_system->actuate();
            // // delay(800);
            // // this->drive_system->update(0.95, 0.95);
            // // this->drive_system->actuate();
            // // delay(200);
            // this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            // this->drive_system->update(0.0, 0.0);
            // this->drive_system->actuate();
            // delay(50000000);
            // this->drive_system->update(0.8, 0.8);
            // this->drive_system->actuate();
            // delay(300);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(-2.8, 0.98);
            this->drive_system->actuate();
            delay(400);
            // this->drive_system->update(-0.1, -0.1);
            // this->drive_system->actuate();
            // delay(300);
            // this->drive_system->update(0.98, 0.98);
            // this->drive_system->actuate();
            // delay(200);
            this->drive_system->update(-0.1, -0.1);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            break;
        // case 1:
            // cheap disable/debounce
            // this->drive_system->update(0.85, 0.85);
            // this->drive_system->actuate();
            // delay(200);
           
        /*
        case 1:
            this->drive_system->update(0.85, -1.8);
            this->drive_system->actuate();
            delay(700);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);
            break;
        */
        case 1:

            this->drive_system->update(0.94, 0.98);
            this->drive_system->actuate();
            delay(100);
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);
            break;

        case 2:

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.93, -3.0);
            this->drive_system->actuate();
            delay(280);
            this->drive_system->update(0.86, 0.86);
            this->drive_system->actuate();
            delay(350);

            for (int i = 0; i < 16; i++) {
                this->drive_system->update(0.93, -0.1);
                this->drive_system->actuate();
                delay(120);
                this->drive_system->update(-0.1, 0.93);
                this->drive_system->actuate();
                delay(120);
            }

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            //TODO UCOMENT PLZZZZZ
            grabCrystal();
            openClaw();

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(-3.0, .98);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_RIGHT);

         
            break;
        
        case 3:

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(400);
            this->drive_system->update(0.93, -3.0);
            this->drive_system->actuate();
            delay(280);
            this->drive_system->update(0.86, 0.86);
            this->drive_system->actuate();
            delay(350);


            for (int i = 0; i < 16; i++) {
                this->drive_system->update(0.93, -0.1);
                this->drive_system->actuate();
                delay(120);
                this->drive_system->update(-0.1, 0.93);
                this->drive_system->actuate();
                delay(120);
            }

            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            //TODO uncoment
            grabCrystal();
            
            this->drive_system->update(-3.0, -3.0);
            this->drive_system->actuate();
            delay(300);
            this->drive_system->update(.98, -3.0);
            this->drive_system->actuate();
            delay(300);
            this->tape_sensor->set_state(MainTapeSensor::FAR_LEFT);

            break;
            
        case 4:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            break;
        
        case 5:
            this->drive_system->update(0.0, 0.0);
            this->drive_system->actuate();
            delay(300);

            break;

        // Gauntlet here
        case 6:
            break;

            //delay(2000); //TODO drop rock here
        // case 4:

        //     this->drive_system->update(0.0, 0.0);
        //     this->drive_system->actuate();
        //     delay(200);
            
        //     this->drive_system->update(-0.80 / 0.3, 0.0);
        //     this->drive_system->actuate();

        //     while (!(this->tape_sensor->qrd2.is_on() || this->tape_sensor->qrd3.is_on())) {
        //         this->tape_sensor->update();
        //     }

        //     break;
    }

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

void IntersectionManager::handle_gauntlet() {

    switch(this->gauntlet_state) {

        case 0:
            
            this->drive_system->update(-1.0, -1.0);
            this->drive_system->actuate();
            delay(550);
            
            this->drive_system->update(-3.5, 0.0);
            this->drive_system->actuate();

            this->tape_sensor->update();
            while (!(this->tape_sensor->qrd2.is_on() || this->tape_sensor->qrd3.is_on())) {
                this->tape_sensor->update();
            }

            gauntlet_timer = millis();

            //this->drive_system->set_speed_add(-0.1);

            this->gauntlet_state++;

            break;

        case 1:
            
            // keep pid going for ____ milliseconds and then increment to next state
            if (millis() - gauntlet_timer >= 1600) {

                this->gauntlet_state++;

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(400);

                this->drive_system->update(-3.0, -3.0);
                this->drive_system->actuate();
                delay(700);

                this->drive_system->set_speed_add(-0.06);

                gauntlet_timer = millis();

                // Set timer for backwards mode
                //gauntlet_timer = millis();

            }

            break;
        
        // case 2:
            
        //     // keep pid going for ____ milliseconds and then increment to next state
        //     if (millis() - gauntlet_timer >= 1600) {

        //         this->gauntlet_state++;

        //         this->drive_system->update(0.0, 0.0);
        //         this->drive_system->actuate();
        //         delay(400);

        //         this->drive_system->update(-3.0, -3.0);
        //         this->drive_system->actuate();
        //         delay(550);

        //         //this->drive_system->set_speed_add(-0.06);

        //         gauntlet_timer = millis();

        //         // Set timer for backwards mode
        //         //gauntlet_timer = millis();

        //     }

        //     break;

        case 2:
            
            // keep pid going for ____ milliseconds and then increment to next state
            if (millis() - gauntlet_timer >= 1600) {

                this->drive_system->set_speed_add(0.0);

                this->gauntlet_state++;

                for (int i = 0; i < 16; i++) {
                    this->drive_system->update(0.93, -0.1);
                    this->drive_system->actuate();
                    delay(120);
                    this->drive_system->update(-0.1, 0.93);
                    this->drive_system->actuate();
                    delay(120);
                }

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(500);

                this->drive_system->update(-3.5, 0.88);
                this->drive_system->actuate();
                delay(280);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(400);

                closeClaw();

                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(false);
                digitalWrite(STEPPERENABLE, HIGH);
                delay(1000);
                openClaw();
                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(true);
                moveZToExtreme(HOME);
                digitalWrite(STEPPERENABLE, HIGH);

                this->drive_system->update(0.94, 0.94);
                this->drive_system->actuate();
                delay(400);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(200);

                this->drive_system->update(0.88, -3.5);
                this->drive_system->actuate();
                delay(280);

                this->drive_system->update(0.0, 0.0);
                this->drive_system->actuate();
                delay(400);

                closeClaw();

                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(false);
                digitalWrite(STEPPERDIR, DOWN);
                for(int i = 0; i < 100; i++) {
                    stepperPulse();
                }
                digitalWrite(STEPPERENABLE, HIGH);

                delay(1000);
                openClaw();
                digitalWrite(STEPPERENABLE, LOW);
                moveZToExtreme(EXTEND);
                homeY(true);
                moveZToExtreme(HOME);
                digitalWrite(STEPPERENABLE, HIGH);

                delay(69420);

            }

            break;
        
    }

}

void IntersectionManager::place_stone() {
    // todo
}
