#ifndef DIGIPOTBASE_H
#define DIGIPOTBASE_H

class DigipotBase {
    public:
        DigipotBase(){}
        virtual ~DigipotBase(){}

        virtual void set_current( int channel, float current )= 0;
        virtual float get_current(int channel)= 0;
        void set_max_current(float c) { max_current= c; }
        void set_factor(float f) { factor= f; }

    protected:
        float factor;
        float max_current;
};


#endif
