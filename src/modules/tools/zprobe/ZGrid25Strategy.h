#ifndef _ZGrid25STRATEGY
#define _ZGrid25STRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <stdint.h>
#include <tuple>

#define ZGrid25_leveling_checksum CHECKSUM("ZGrid25-leveling")

class StreamOutput;
class Plane3D;

class ZGrid25Strategy : public LevelingStrategy
{
public:
    ZGrid25Strategy(ZProbe *zprobe);
    ~ZGrid25Strategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    void homexyz();

    void move(float *position, float feed);
    void next_cal(void);

    void setAdjustFunction(bool);
    bool doProbing(StreamOutput *stream);   

    bool loadGrid();
    bool saveGrid();

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float, float> parseXYZ(const char *str);

    uint16_t numRows;
    uint16_t numCols;
    float *pData;
        
    float slow_rate;
    float bed_x;
    float bed_y;
    float bed_div_x;
    float bed_div_y;
    float cal[3];            // calibration positions for manual leveling
    struct {
        bool home:1;
        bool save:1;
        bool is_scara:1;
        bool in_cal:1;
    };
};

#endif
