// Generic calibration strategy based on nonlinear optimization
//
// Copyright (c) Oskar Linde 2015
//
// Licence: GPL v2

#ifndef _GENERICLEVELINGSTRATEGY
#define _GENERICLEVELINGSTRATEGY

#include "LevelingStrategy.h"
#include "Vector3.h"
#include <string>
#include <vector>

#define calibration_strategy_checksum CHECKSUM("calibration")

class CalibrationStrategy : public LevelingStrategy
{
public:
    CalibrationStrategy(class ZProbe *zprobe) : LevelingStrategy(zprobe){}
    ~CalibrationStrategy(){}
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    // Radius of probe circle. TODO: model other build plate shapes
    float probe_radius;

    // cache endstop trim values
    float trim[3]{};

    // sum of home_offset and homing_position
    float home_position[3]{};

    // store output stream during calibration
    class StreamOutput * output_stream;


    // impl details
    bool  setup_probe();
    bool  optimize_model(int n, int repeats, const std::string &parameters, class StreamOutput *stream);
    float get_parameter(char parameter);
    bool  set_parameter(char parameter, float value);
    bool  update_parameter(char parameter, float delta);
    void  compute_JTJ_JTr(std::vector<Vector3> const& actuator_positions,
                          std::string          const& parameters,
                          std::vector<float>        & JTJ,
                          std::vector<float>        & JTr,
                          std::vector<float>        & scratch);
    void  compute_cartesian_position(float const actuator_position[3], float cartesian_position[3]);
    float compute_model_error(float const actuator_position[3]);
    float compute_model_rms_error(std::vector<Vector3> const& actuator_positions);
    void  update_compensation_transformation();
    void  init_home_position();

    // probes n points, return actuator positions of trigger points
    bool  probe_pattern(int n, int repeats, float actuator_positions[/*N*/][3]);
    bool  probe_spiral(int n, int repeats, float actuator_positions[/*N*/][3]);
    bool  probe_symmetric(int n, int repeats, float actuator_positions[/*N*/][3]);
};

#endif
