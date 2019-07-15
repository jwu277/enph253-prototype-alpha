#include <Arduino.h>

#include "sensors/QrdSensor.hpp"
#include "sensors/MainTapeSensor.hpp"

#include <vector>

// Current operation:
// x-axis: x = 0 at centered, left-to-right
// For x = 0, tape between sensors (not under any)
// For x = -+ 1, tape only beneath right/left, respectively
// For x = -+ FAR_OFF, tape not under any: use state machine

// How far off the tape follower would be if both sensors are off tape
// (could be computed as 1 + tape width / sensor width), units of sensor widths

using namespace std;

// Constructor
MainTapeSensor::MainTapeSensor(vector<PinName> pins,
    vector<tuple<int, int>> calibration, vector<double> weights)
    : qrd1(QrdSensor(PA_3, make_tuple(50, 450))), qrd2(QrdSensor(PA_7, make_tuple(50, 450))),
    qrd3(QrdSensor(PA_6, make_tuple(50, 450))), qrd4(QrdSensor(PA_5, make_tuple(50, 450))),
    qrd5(QrdSensor(PA_4, make_tuple(50, 450))), qrd6(QrdSensor(PA_0, make_tuple(50, 450))) {

    this->create_qrds(pins, calibration);

    this->weights = weights;
    
    this->state = CENTRE; // start neutral
    this->x = 0.0; // default value of 0.0

    this->qrd0_status = false;
    this->qrd7_status = false;

    // TODO: clean up
    this->init_sensor_weights();

}

void MainTapeSensor::create_qrds(vector<PinName> pins, vector<tuple<int, int>> calibration) {

    this->qrds = {};

    for (unsigned i = 0; i < pins.size(); i++) {
        this->qrds.push_back(QrdSensor(pins.at(i), calibration.at(i)));
    }

}

void MainTapeSensor::init() {

    for (QrdSensor qrd : this->qrds) {
        qrd.init();
    }

    pinMode(PA11, INPUT_PULLUP);
    pinMode(PA15, INPUT_PULLUP);

}

// Read data
void MainTapeSensor::update() {

    MainTapeSensor::update_qrds();
    this->qrd0_status = digitalRead(PA11);
    this->qrd7_status = digitalRead(PA15);
    MainTapeSensor::update_state();

}

void MainTapeSensor::update_qrds() {
    for (QrdSensor qrd : this->qrds) {
        qrd.update();
    }
    this->qrd1.update();
    this->qrd2.update();
    this->qrd3.update();
    this->qrd4.update();
    this->qrd5.update();
    this->qrd6.update();
}

void MainTapeSensor::update_state() {

    this->x = 0.0;

    /*
    for (int i = 0; i < this->qrds.size(); i++) {
        this->x += this->qrds.at(i).get_read() * this->weights.at(i);
        //Serial.print(this->qrds.at(i).get_read());
        //Serial.print(this->qrds.at(i).get_value());
        //Serial.print("       ");
    }
    */

   this->x += this->qrd1.get_read() * this->qrd1_weight;
   this->x += this->qrd2.get_read() * this->qrd2_weight;
   this->x += this->qrd3.get_read() * this->qrd3_weight;
   this->x += this->qrd4.get_read() * this->qrd4_weight;
   this->x += this->qrd5.get_read() * this->qrd5_weight;
   this->x += this->qrd6.get_read() * this->qrd6_weight;

   if (this->x > 0) {
       this->state = RIGHT;
   }
   else if (this->x < 0) {
       this->state = LEFT;
   }

   //pwm_start(PA_0, 1000000, 10, 0, 0);
   //pwm_start(PA_8, 1000000, 10, 0, 0);

   if (!this->qrd1.is_on() && !this->qrd2.is_on() && !this->qrd3.is_on() &&
        !this->qrd4.is_on() && !this->qrd5.is_on() && !this->qrd6.is_on()) {
       if (this->state == LEFT || this->state == FAR_LEFT) {
           this->x = -4.0;
           this->state = FAR_LEFT;
           //pwm_start(PA_0, 1000000, 10, 10, 0);
       }
       if (this->state == RIGHT || this->state == FAR_RIGHT) {
           this->x = 4.0;
           this->state = FAR_RIGHT;
           //pwm_start(PA_8, 1000000, 10, 10, 0);
       }
   }

}

bool MainTapeSensor::is_both_on() {
    return false;
}

// For PID
double* MainTapeSensor::get_x_ptr() {
    return &(this->x);
}

bool MainTapeSensor::is_far_left() {
    return this->state == FAR_LEFT;
}

bool MainTapeSensor::is_far_right() {
    return this->state == FAR_RIGHT;
}

void MainTapeSensor::set_state(State state) {
    this->state = state;
}

vector<bool> MainTapeSensor::get_qrds_status() {

    // TODO: update + test

    vector<bool> status = {};

    status.push_back(this->qrd0_status);
    status.push_back(this->qrd1.is_on());
    status.push_back(this->qrd2.is_on());
    status.push_back(this->qrd3.is_on());
    status.push_back(this->qrd4.is_on());
    status.push_back(this->qrd5.is_on());
    status.push_back(this->qrd6.is_on());
    status.push_back(this->qrd7_status);

    // temp print
    for (vector<bool>::iterator it = status.begin(); it != status.end(); it++) {
        //Serial.print(*it ? 1 : 0);
        //Serial.print("     ");
    }
    //Serial.println();

    return status;

}

void MainTapeSensor::init_sensor_weights() {

    this->qrd1_weight = 1.5;
    this->qrd2_weight = 1.0;
    this->qrd3_weight = 0.5;
    this->qrd4_weight = -0.5;
    this->qrd5_weight = -1.0;
    this->qrd6_weight = -1.5;

}

void MainTapeSensor::ignore_right_sensors() {
    this->qrd4_weight = 0.0;
    this->qrd5_weight = 0.0;
    this->qrd6_weight = 0.0;
}
