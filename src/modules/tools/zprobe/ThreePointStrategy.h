#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"
#include "Plane3D.h"

#include <string.h>
#include <tuple>
#include <cmath>

#define three_point_leveling_strategy_checksum CHECKSUM("three-point-leveling")

class StreamOutput;

class ThreePointStrategy : public LevelingStrategy
{
public:
    ThreePointStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){ for (int i = 0; i < 3; ++i) {probe_points[i]= std::make_tuple(NAN,NAN);}; plane= nullptr; }
    ~ThreePointStrategy(){ delete plane; }
    bool handleGcode(Gcode* gcode);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    bool doProbing(StreamOutput *stream);
    std::tuple<float, float> parseXY(const char *str);

    std::tuple<float, float> probe_points[3];
    Plane3D *plane;
};

#endif
