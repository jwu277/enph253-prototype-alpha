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
MainTapeSensor::MainTapeSensor()
    : qrd0(QrdSensor(PA_6, make_tuple(50, 450))),
    qrd1(QrdSensor(PA_5, make_tuple(50, 450))), qrd2(QrdSensor(PA_3, make_tuple(50, 450))),
    qrd3(QrdSensor(PA_2, make_tuple(50, 450))), qrd4(QrdSensor(PA_1, make_tuple(50, 450))),
    qrd5(QrdSensor(PA_0, make_tuple(50, 450))), qrd6(QrdSensor(PA_4, make_tuple(50, 450))),
    qrd7(QrdSensor(PA_7, make_tuple(50, 450))) {

    this->weights = weights;
    
    this->state = CENTRE; // start neutral
    this->x = 0.0; // default value of 0.0

    this->qrd0_status = false;
    this->qrd7_status = false;

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

    this->qrd0.set_on_threshold(520);
    this->qrd7.set_on_threshold(520);

}

// Read data
void MainTapeSensor::update() {
    MainTapeSensor::update_qrds();
    MainTapeSensor::update_state();
}

void MainTapeSensor::update_qrds() {
    for (QrdSensor qrd : this->qrds) {
        qrd.update();
    }
    this->qrd0.update();
    this->qrd1.update();
    this->qrd2.update();
    this->qrd3.update();
    this->qrd4.update();
    this->qrd5.update();
    this->qrd6.update();
    this->qrd7.update();
}

void MainTapeSensor::update_state() {

    this->x = 0.0;

    this->x += this->qrd1.get_read() * this->qrd1_weight;
    this->x += this->qrd2.get_read() * this->qrd2_weight;
    this->x += this->qrd3.get_read() * this->qrd3_weight;
    this->x += this->qrd4.get_read() * this->qrd4_weight;
    this->x += this->qrd5.get_read() * this->qrd5_weight;
    this->x += this->qrd6.get_read() * this->qrd6_weight;

   if (this->qrd1.is_on() && !this->qrd2.is_on()) {
       this->x = 2 * this->qrd1.get_read() * this->qrd1_weight;
   }
   else if (this->qrd6.is_on() && !this->qrd5.is_on()) {
       this->x = 2 * this->qrd6.get_read() * this->qrd6_weight;
   }

   if (this->x > 0) {
       this->state = RIGHT;
   }
   else if (this->x < 0) {
       this->state = LEFT;
   }
   if (!this->qrd1.is_on() && !this->qrd2.is_on() && !this->qrd3.is_on() &&
        !this->qrd4.is_on() && !this->qrd5.is_on() && !this->qrd6.is_on()) {
       if (this->state == LEFT || this->state == FAR_LEFT) {
           this->x = -8.0;
           this->state = FAR_LEFT;
       }
       if (this->state == RIGHT || this->state == FAR_RIGHT) {
           this->x = 8.0;
           this->state = FAR_RIGHT;
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

    vector<bool> status = {};

    status.push_back(this->qrd0.is_on());
    status.push_back(this->qrd1.is_on());
    status.push_back(this->qrd2.is_on());
    status.push_back(this->qrd3.is_on());
    status.push_back(this->qrd4.is_on());
    status.push_back(this->qrd5.is_on());
    status.push_back(this->qrd6.is_on());
    status.push_back(this->qrd7.is_on());

    return status;
}

void MainTapeSensor::init_sensor_weights() {

    this->qrd1_weight = 3.0;
    this->qrd2_weight = 2.0;
    this->qrd3_weight = 1.0;
    this->qrd4_weight = -1.0;
    this->qrd5_weight = -2.0;
    this->qrd6_weight = -3.0;

}

void MainTapeSensor::reset_thresholds() {
    this->qrd0.set_on_threshold(250);
    this->qrd1.set_on_threshold(250);
    this->qrd2.set_on_threshold(250);
    this->qrd3.set_on_threshold(250);
    this->qrd4.set_on_threshold(250);
    this->qrd5.set_on_threshold(250);
    this->qrd6.set_on_threshold(250);
    this->qrd7.set_on_threshold(250);
}

void MainTapeSensor::ignore_right_sensors(int start) {

    switch (start) {

        case 1:
            this->qrd1_weight = 0.0;
        case 2:
            this->qrd2_weight = 0.0;
        case 3:
            this->qrd3_weight = 0.0;
        case 4:
            this->qrd4_weight = 0.0;
        case 5:
            this->qrd5_weight = 0.0;
        case 6:
            this->qrd6_weight = 0.0;

    }

}
