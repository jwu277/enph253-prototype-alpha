#ifndef BASESENSOR
#define BASESENSOR

// BaseSensor is an abstract base class for sensors
class BaseSensor {

    public:

        // Hardware read of sensor
        // To be put in loop()
        virtual int read() = 0;

};

#endif
