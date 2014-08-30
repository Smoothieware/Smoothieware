#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define three_point_leveling_strategy_checksum CHECKSUM("three-point-leveling")

class StreamOutput;
class Plane3D;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} arm_bilinear_interp_instance;

class GridStrategy : public LevelingStrategy
{
public:
    GridStrategy(ZProbe *zprobe);
    ~GridStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

    arm_bilinear_interp_instance bed_level_data;




private:
    void home();

    void move(float *position, float feed);
    void next_cal(void);

    float slow_rate;
    float cal[3];            // calibration positions for manual leveling: TODO - remove when auto is functional
    bool in_cal;

    struct {
        bool home:1;
        bool save:1;
        bool is_scara:1;
    };
};

#endif
