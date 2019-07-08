#ifndef SIDETAPESENSOR
#define SIDETAPESENSOR

#include "QrdSensor.hpp"
#include "BaseSensor.hpp"


// SideTapeSensor represents the side tape-sensing system
// for detecting intersections
class SideTapeSensor: public BaseSensor {

    private:

        QrdSensor left_side_qrd;
        QrdSensor right_side_qrd;

        bool left_on;
        bool right_on;

        void update_qrds();
        void update_state();

    public:

        // Constructor
        SideTapeSensor(PinName left_qrd_pin, PinName right_qrd_pin);

        void init();

        // Update tape sensor
        void update();

        bool is_left_on();
        bool is_right_on();

};

#endif
