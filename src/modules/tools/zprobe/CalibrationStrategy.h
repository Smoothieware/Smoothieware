// Generic calibration strategy based on nonlinear optimization
//
// Copyright (c) Oskar Linde 2015
//
// Licence: GPL v2

#ifndef _GENERICLEVELINGSTRATEGY
#define _GENERICLEVELINGSTRATEGY

#include "LevelingStrategy.h"
#include <string>

#define calibration_strategy_checksum CHECKSUM("calibration")

class CalibrationStrategy : public LevelingStrategy
{
public:
    CalibrationStrategy(class ZProbe *zprobe) : LevelingStrategy(zprobe){}
    ~CalibrationStrategy(){}
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

    // probes n points in a spiral pattern, return actuator positions of trigger points
    bool probe_spiral(int n, int repeats, float actuator_positions[/*N*/][3]);
private:
    bool setup_probe();
    
    float probe_radius;
    bool optimize_delta_model(int n, int repeats, const std::string &parameters, class StreamOutput *stream);
};

#endif
