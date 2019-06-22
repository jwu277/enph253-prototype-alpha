#ifndef POLARITYACTUATOR
#define POLARITYACTUATOR

#include "BaseActuator.hpp"

// PolarityActuator is an abstract base class for actuators with a polarity.
// That is, there is a distinction between a forward and reverse mode/motion.
class PolarityActuator: public BaseActuator {

    protected:

        // dir is the current direction of the actuator

        enum Direction {
            FORWARD,
            REVERSE
        };

        Direction dir;

};

#endif
