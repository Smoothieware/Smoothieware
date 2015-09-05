// Generic calibration strategy based on nonlinear optimization
//
// Copyright (c) Oskar Linde 2015
//
// Licence: GPL v2

#ifndef _GENERICLEVELINGSTRATEGY
#define _GENERICLEVELINGSTRATEGY

#include "LevelingStrategy.h"
#include <string>
#include <vector>

#define calibration_strategy_checksum CHECKSUM("calibration")
struct V3 { float m[3]; };

class CalibrationStrategy : public LevelingStrategy
{
public:
    CalibrationStrategy(class ZProbe *zprobe) : LevelingStrategy(zprobe){}
    ~CalibrationStrategy(){}
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

    // probes n points, return actuator positions of trigger points
    bool probe_pattern(int n, int repeats, float actuator_positions[/*N*/][3]);

private:
    // Radius of probe circle. TODO: model other build plate shapes
    float probe_radius;

    // Plane tilt
    float plane_p = 0, plane_q = 0;

    // Separate plane offset variable (to allow Home Offset to be kept constant)
    float plane_offset = 0;

    // Last row of plane rotation matrix
    float inverse_r[3] = {0,0,1};

    // cache endstop trim values
    float trim[3] = {};


    // impl details
    bool  setup_probe();
    bool  optimize_model(int n, int repeats, const std::string &parameters, class StreamOutput *stream);
    float get_parameter(char parameter);
    bool  set_parameter(char parameter, float value);
    bool  update_parameter(char parameter, float delta);
    void  compute_JTJ_JTr(std::vector<V3> const& actuator_positions,
                          std::string     const& parameters,
                          std::vector<float>   & JTJ,
                          std::vector<float>   & JTr,
                          std::vector<float>   & scratch);
    float compute_model_error(float const actuator_position[3]);
    float compute_model_rms_error(std::vector<V3> const& actuator_positions);
    void  update_compensation_transformation();
    bool  probe_spiral(int n, int repeats, float actuator_positions[/*N*/][3]);
    bool  probe_symmetric(int n, int repeats, float actuator_positions[/*N*/][3]);
};

#endif
