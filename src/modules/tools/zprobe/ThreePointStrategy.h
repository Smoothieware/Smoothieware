#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>

#define three_point_leveling_strategy_checksum CHECKSUM("three-point-leveling")

class StreamOutput;

class ThreePointStrategy : public LevelingStrategy
{
public:
    ThreePointStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){ memset(probe_points, 0, sizeof probe_points); }
    ~ThreePointStrategy(){}
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    float probe_points[3][2];
};

#endif
