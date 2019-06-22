#ifndef POLARITYMOTOR
#define POLARITYMOTOR

#include "BaseActuator.hpp"

class PolarityMotor: public BaseActuator {

    protected:

        enum Direction {
            FORWARD,
            REVERSE
        };

        // 0: BACKWARDS, 1: FORWARD
        Direction dir;

};

#endif
