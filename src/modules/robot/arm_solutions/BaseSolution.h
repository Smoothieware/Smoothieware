// Base class for an arm solution, only usefull for inheritence. http://en.wikipedia.org/wiki/Arm_solution
#ifndef BASESOLUTION_H
#define BASESOLUTION_H

class Config;

class BaseSolution {
    public:
        BaseSolution(){};
        BaseSolution(Config*){};

        virtual ~BaseSolution() {};

        virtual void cartesian_to_actuator( float[], float[] ) = 0;
        virtual void actuator_to_cartesian( float[], float[] ) = 0;

        virtual bool set_optional(char parameter, float  value) { return false; };
        virtual bool get_optional(char parameter, float *value) { return false; };

        virtual bool confirm_homing_set(uint8_t actuators[]) = 0;
};

#endif
