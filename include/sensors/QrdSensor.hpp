#ifndef QRDSENSOR
#define QRDSENSOR

#include "sensors/AnalogSensor.hpp"

using namespace std;

// QRDSensor is a concrete QRD Sensor
class QrdSensor: public AnalogSensor {

    private:

        // Tape + white thresholds for calibration
        int on_tape;
        int on_white;

        int threshold;

    public:

        // Constructor for QRD Sensor
        QrdSensor(PinName pin, tuple<int, int> thresholds);

        void init();

        double get_read();

        bool is_on();

        void set_on_threshold(int thresh);

};

#endif
