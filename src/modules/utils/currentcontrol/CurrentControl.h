#ifndef CURRENTCONTROL_H
#define CURRENTCONTROL_H

#include "Module.h"

class DigipotBase;

class CurrentControl : public Module {
    public:
        CurrentControl();
        virtual ~CurrentControl() {};

        void on_module_loaded();
        void on_gcode_received(void *);

    private:
        DigipotBase* digipot;

};





#endif
