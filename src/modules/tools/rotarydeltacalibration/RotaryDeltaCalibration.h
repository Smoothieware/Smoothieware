#pragma once

#include "Module.h"

class Gcode;

class RotaryDeltaCalibration : public Module
{
public:
    RotaryDeltaCalibration(){};
    virtual ~RotaryDeltaCalibration(){};

    void on_module_loaded();

private:
    void on_gcode_received(void *argument);
    bool get_homing_offset(float*);
};
