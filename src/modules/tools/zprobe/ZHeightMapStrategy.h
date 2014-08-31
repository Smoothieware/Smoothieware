#ifndef _ZHEIGHTMAPSTRATEGY
#define _ZHEIGHTMAPSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <stdint.h>

#define zheightmap_leveling_checksum CHECKSUM("zheightmap-leveling")

class StreamOutput;
//class Plane3D;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} arm_bilinear_interp_instance;

class ZHeightMapStrategy : public LevelingStrategy
{
public:
    ZHeightMapStrategy(ZProbe *zprobe);
    ~ZHeightMapStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

    arm_bilinear_interp_instance bed_level_data;

private:
    void homexyz();

    void move(float *position, float feed);
    void next_cal(void);

    float arm_bilinear_interp(float X, float Y);

    void setAdjustFunction(bool);

    float slow_rate;
    float bed_x;
    float bed_y;
    float bed_div_x;
    float bed_div_y;
    float cal[3];            // calibration positions for manual leveling: TODO - remove when auto is functional

    struct {
        bool home:1;
        bool save:1;
        bool is_scara:1;
        bool in_cal:1;
    };
};

#endif
