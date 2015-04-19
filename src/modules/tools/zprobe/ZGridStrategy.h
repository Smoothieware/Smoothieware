#ifndef _ZGridSTRATEGY
#define _ZGridSTRATEGY

#include "LevelingStrategy.h"

#include <string>
#include <stdint.h>
#include <tuple>

#define ZGrid_leveling_checksum CHECKSUM("ZGrid-leveling")

class StreamOutput;

class ZGridStrategy : public LevelingStrategy
{
public:
    ZGridStrategy(ZProbe *zprobe);
    ~ZGridStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    void homexyz();

    void move(float *position, float feed);
    void next_cal(void);
    float getZhomeoffset();
    void setZoffset(float zval);

    void setAdjustFunction(bool);
    bool doProbing(StreamOutput *stream);
    void normalize_grid();

    bool loadGrid(std::string args);
    bool saveGrid(std::string args);
    void calcConfig();

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float, float> parseXYZ(const char *str);

    uint16_t numRows;
    uint16_t numCols;
    float *pData;

    bool wait_for_probe;

    float slow_rate;
    float bed_x;
    float bed_y;
    float bed_z;
    float cal_offset_x;
    float cal_offset_y;
    float bed_div_x;
    float bed_div_y;
    float cal[3];            // calibration positions for manual leveling
    struct {
        bool in_cal:1;
        bool center_zero:1;
        bool circular_bed:1;
    };
};

#endif
