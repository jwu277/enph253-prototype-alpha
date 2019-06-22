#ifndef BASEACTUATOR
#define BASEACTUATOR

class BaseActuator {

    public:
        virtual void init() = 0;
        virtual void actuate() = 0;

};

#endif
