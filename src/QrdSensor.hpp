#ifndef QRDSENSOR
#define QRDSENSOR

#include "AnalogSensor.hpp"

// QRDSensor is a concrete QRD Sensor
class QrdSensor: public AnalogSensor {

    public:

        // Constructor for QRD Sensor
        QrdSensor(PinName pin);

        void init();

        bool is_on();


};

#endif
