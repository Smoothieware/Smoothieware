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

    bool doProbe(Gcode *gc, bool scanonly);
    bool findBed();
    void setAdjustFunction(bool on);
    void print_bed_level(StreamOutput *stream);
    void doCompensation(float *target, bool inverse);
    void reset_bed_level();
    void save_grid(StreamOutput *stream);
    bool load_grid(StreamOutput *stream);

    float initial_height;
    float tolerance;

    float height_limit;
    float dampening_start;
    float damping_interval;

    float *grid;
    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float, float> m_attach;
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
        bool do_manual_attach:1;
        bool only_by_two_corners:1;
        bool human_readable:1;
        bool new_file_format:1;
    };
};
