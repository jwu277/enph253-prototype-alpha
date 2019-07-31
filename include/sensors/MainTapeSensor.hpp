#ifndef MAINTAPESENSOR
#define MAINTAPESENSOR

#include <vector>

#include "sensors/BaseSensor.hpp"
#include "sensors/QrdSensor.hpp"

using namespace std;

// MainTapeSensor is the main tape-sensing system, composite of QRD sensors
class MainTapeSensor: public BaseSensor {

    public:

        // Constructor
        MainTapeSensor();

        QrdSensor qrd0;
        QrdSensor qrd1;
        QrdSensor qrd2;
        QrdSensor qrd3;
        QrdSensor qrd4;
        QrdSensor qrd5;
        QrdSensor qrd6;
        QrdSensor qrd7;

        void init();

        // Update tape sensors
        void update();

        bool is_both_on();

        double* get_x_ptr();

        // state machine
        
        enum State {
            FAR_LEFT,
            LEFT,
            CENTRE,
            RIGHT,
            FAR_RIGHT
        };

        bool is_far_left();
        bool is_far_right();

        void set_state(State state); 

        vector<bool> get_qrds_status();

        void init_sensor_weights();
        void ignore_left_sensors();
        void ignore_right_sensors();

        void reset_thresholds();

    private:

        //QrdSensor left_qrd;
        //QrdSensor right_qrd;

        vector<QrdSensor> qrds;

        double qrd0_weight;
        double qrd1_weight;
        double qrd2_weight;
        double qrd3_weight;
        double qrd4_weight;
        double qrd5_weight;
        double qrd6_weight;
        double qrd7_weight;

        // temp
        bool qrd0_status;
        bool qrd7_status;

        State state;

        vector<double> weights;

        // PID error term
        // Corresponds to how far off the tape we are
        // x = 0 is centered
        double x;

        // Constructor for qrds
        void create_qrds(vector<PinName> pins, vector<tuple<int, int>> calibration);

        void update_qrds();

        void update_state();

};

#endif
