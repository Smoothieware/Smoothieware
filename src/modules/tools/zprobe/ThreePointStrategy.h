#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define three_point_leveling_strategy_checksum CHECKSUM("three-point-leveling")

class StreamOutput;
class Plane3D;

class ThreePointStrategy : public LevelingStrategy
{
public:
    ThreePointStrategy(ZProbe *zprobe);
    ~ThreePointStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    void homeXY();
    bool doProbing(StreamOutput *stream);
    std::tuple<float, float> parseXY(const char *str);
    std::tuple<float, float, float> parseXYZ(const char *str);
    void setAdjustFunction(bool);

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float> probe_points[3];
    Plane3D *plane;
    struct {
        bool home:1;
        bool save:1;
    };
    float tolerance;
};

#endif
