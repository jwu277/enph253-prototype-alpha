#ifndef BASEACTUATOR
#define BASEACTUATOR

// BaseActuator is an abstract base class for actuators
class BaseActuator {

    public:

        // Hardware initialization for actuator.
        // init() method of each actuator should be called in setup()
        virtual void init() = 0;

        // Hardware execution of actuator.
        // Intended to be called in loop()
        virtual void actuate() = 0;

};

#endif
