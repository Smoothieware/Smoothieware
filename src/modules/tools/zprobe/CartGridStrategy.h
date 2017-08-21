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
    bool findBed();
    void setAdjustFunction(bool on);
    void print_bed_level(StreamOutput *stream);
    void doCompensation(float *target, bool inverse);
    void reset_bed_level();
    void save_grid(StreamOutput *stream);
    bool load_grid(StreamOutput *stream);
    bool probe_grid(int n, int m, float _x_start, float _y_start, float _x_size, float _y_size, StreamOutput *stream);

    float initial_height;
    float tolerance;

    float *grid;
    std::tuple<float, float, float> probe_offsets;
    float x_start,y_start;
    float x_size,y_size;

    struct {
        uint8_t configured_grid_x_size:8;
        uint8_t configured_grid_y_size:8;
        uint8_t current_grid_x_size:8;
        uint8_t current_grid_y_size:8;
    };

    struct {
        bool save:1;
        bool do_home:1;
        bool only_by_two_corners:1;
        bool human_readable:1;
    };
};
