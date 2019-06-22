#ifndef POLARITYMOTOR
#define POLARITYMOTOR

#include "BaseActuator.hpp"

// PolarityMotor is an abstract base class for motors with a polarity.
// That is, there is a distinction between a forward and reverse mode/motion.
class PolarityMotor: public BaseActuator {

    protected:

        // dir is the current direction of the motor

        enum Direction {
            FORWARD,
            REVERSE
        };

        Direction dir;

};

#endif
