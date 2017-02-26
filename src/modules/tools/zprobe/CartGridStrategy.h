#pragma once

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define cart_grid_leveling_strategy_checksum CHECKSUM("rectangular-grid")

class StreamOutput;
class Gcode;

class CartGridStrategy : public LevelingStrategy
{
public:
    CartGridStrategy(ZProbe *zprobe);
    ~CartGridStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:

    bool doProbe(Gcode *gc);
    float findBed();
    void setAdjustFunction(bool on);
    void print_bed_level(StreamOutput *stream);
    void doCompensation(float *target, bool inverse);
    void reset_bed_level();
    void save_grid(StreamOutput *stream);
    bool load_grid(StreamOutput *stream);
    bool probe_grid(int n, float x_size, float y_size, StreamOutput *stream);

    float initial_height;
    float tolerance;

    float *grid;
    std::tuple<float, float, float> probe_offsets;
    uint8_t grid_size;
    float x_size,y_size;

    struct {
        bool save:1;
        bool do_home:1;
    };
};
