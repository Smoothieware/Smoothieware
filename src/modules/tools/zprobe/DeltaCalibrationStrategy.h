#ifndef _DELTALEVELINGSTRATEGY
#define _DELTALEVELINGSTRATEGY

#include "LevelingStrategy.h"

#define delta_calibration_strategy_checksum CHECKSUM("delta-calibration")

class StreamOutput;

class DeltaCalibrationStrategy : public LevelingStrategy
{
public:
    DeltaCalibrationStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){};
    ~DeltaCalibrationStrategy(){};
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    bool set_trim(float x, float y, float z, StreamOutput *stream);
    bool get_trim(float& x, float& y, float& z);
    bool calibrate_delta_endstops(Gcode *gcode);
    bool calibrate_delta_radius(Gcode *gcode);
    bool probe_delta_points(Gcode *gcode);

    float probe_radius;
};

#endif
