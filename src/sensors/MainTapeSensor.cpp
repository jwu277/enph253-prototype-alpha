#include <Arduino.h>

#include "sensors/QrdSensor.hpp"
#include "sensors/MainTapeSensor.hpp"

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
    : qrd1(QrdSensor(PA_7, make_tuple(50, 150))),  qrd2(QrdSensor(PA_6, make_tuple(50, 150))),
    qrd3(QrdSensor(PA_5, make_tuple(50, 150))), qrd4(QrdSensor(PA_4, make_tuple(50, 150))){

    this->create_qrds(pins, calibration);

    this->weights = weights;
    
    this->state = CENTRE; // start neutral
    this->x = 0.0; // default value of 0.0

}

void MainTapeSensor::create_qrds(vector<PinName> pins, vector<tuple<int, int>> calibration) {

    this->qrds = {};

    for (int i = 0; i < pins.size(); i++) {
        this->qrds.push_back(QrdSensor(pins.at(i), calibration.at(i)));
    }

}

void MainTapeSensor::init() {

    for (QrdSensor qrd : this->qrds) {
        qrd.init();
    }

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
    this->qrd1.update();
    this->qrd2.update();
    this->qrd3.update();
    this->qrd4.update();
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

   this->x += this->qrd1.get_read() * 1.5;
   this->x += this->qrd2.get_read() * 0.5;
   this->x += this->qrd3.get_read() * -0.5;
   this->x += this->qrd4.get_read() * -1.5;

   if (this->x > 0) {
       this->state = RIGHT;
   }
   else if (this->x < 0) {
       this->state = LEFT;
   }

   if (!this->qrd1.is_on() && !this->qrd2.is_on() && !this->qrd3.is_on() && !this->qrd4.is_on()) {
       if (this->state == LEFT || this->state == FAR_LEFT) {
           this->x = -3.0;
           this->state = FAR_LEFT;
       }
       if (this->state == RIGHT || this->state == FAR_RIGHT) {
           this->x = 3.0;
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
