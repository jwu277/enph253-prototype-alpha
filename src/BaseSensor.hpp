#ifndef BASESENSOR
#define BASESENSOR

// BaseSensor is an abstract base class for sensors
class BaseSensor {

    public:
        
        // Update device state with a reading
        virtual void update() = 0;

};

#endif
