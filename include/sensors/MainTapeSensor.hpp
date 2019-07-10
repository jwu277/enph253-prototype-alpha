#ifndef MAINTAPESENSOR
#define MAINTAPESENSOR

#include "sensors/BaseSensor.hpp"
#include "sensors/QrdSensor.hpp"


// MainTapeSensor is the main tape-sensing system, composite of QRD sensors
class MainTapeSensor: public BaseSensor {

    public:

        // Constructor
        MainTapeSensor(PinName left_qrd_pin, PinName right_qrd_pin);

        void init();

        // Update tape sensor
        void update();

        bool is_both_on();

        double* get_x_ptr();

        // state machine
        enum State {
            LEFT,
            CENTRE,
            RIGHT
        };

        void set_state(State state);

    private:

        QrdSensor left_qrd;
        QrdSensor right_qrd;

        State state;

        void update_qrds();

        void update_state();

        // PID error term
        // Corresponds to how far off the tape we are
        // x = 0 is centered

        double x;

};

#endif