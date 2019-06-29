#ifndef TAPESENSOR
#define TAPESENSOR

#include "QrdSensor.hpp"
#include "BaseSensor.hpp"


// TapeSensor is the main tape-sensing system, composite of QRD sensors
class TapeSensor: public BaseSensor {

    private:

        QrdSensor left_qrd;
        QrdSensor right_qrd;

        // PID error term
        // Corresponds to how far off the tape we are
        // x = 0 is centered
        double x;

        // state machine
        enum State {
            LEFT,
            CENTRE,
            RIGHT
        };

        State state;

        void update_qrds();

        void update_state();

    public:

        // Constructor: includes the two QRDs
        TapeSensor(PinName left_qrd_pin, PinName right_qrd_pin);

        // Update tape sensor
        void update();

        // Get x
        double get_x();

};

#endif
