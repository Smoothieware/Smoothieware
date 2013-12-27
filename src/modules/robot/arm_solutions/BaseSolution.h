// Base class for an arm solution, only usefull for inheritence. http://en.wikipedia.org/wiki/Arm_solution
#ifndef BASESOLUTION_H
#define BASESOLUTION_H

class BaseSolution {
    public:
        BaseSolution();
        virtual ~BaseSolution() {};
        virtual void millimeters_to_steps( float millimeters[], int steps[] ) = 0;
        virtual void steps_to_millimeters( int steps[], float millimeters[] ) = 0;

        virtual void set_steps_per_millimeter( float steps[] ) = 0;
        virtual void get_steps_per_millimeter( float steps[] ) = 0;

        virtual bool set_optional(char parameter, float value) { return false; };
        virtual bool get_optional(char parameter, float *value) { return false; };
};

#endif
