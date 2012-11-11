// Base class for an arm solution, only usefull for inheritence. http://en.wikipedia.org/wiki/Arm_solution
#ifndef BASESOLUTION_H
#define BASESOLUTION_H

class BaseSolution {
    public:
        BaseSolution();
        virtual void millimeters_to_steps( double millimeters[], int steps[] ) = 0;
        virtual void steps_to_millimeters( int steps[], double millimeters[] ) = 0;
};

#endif
