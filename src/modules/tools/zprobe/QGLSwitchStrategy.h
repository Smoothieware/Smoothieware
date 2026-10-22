#ifndef _QGLSWITCHSTRATEGY
#define _QGLSWITCHSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define quad_gantry_leveling_strategy_checksum CHECKSUM("quad-gantry-leveling")

#define X_  X_AXIS
#define Y_  Y_AXIS

#define Z__ 0
#define Z1_ 1
#define Z2_ 2
#define Z3_ 3


class StreamOutput;

class QGLSwitchStrategy : public LevelingStrategy
{
public:
    QGLSwitchStrategy(ZProbe *zprobe);
    ~QGLSwitchStrategy();
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    void homeXY();
    bool doProbing(StreamOutput *stream);
    std::tuple<float, float> parseXY(const char *str);
    std::tuple<float, float, float> parseXYZ(const char *str);
    void setAdjustFunction(bool);
    bool test_probe_points(Gcode *gcode);

    // turn the switch on or off
    void set_switch(uint16_t switch_checksum, bool switch_state);

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float> probe_points[4];
    std::tuple<float, float> gantry_corner0;
    std::tuple<float, float> gantry_corner2;

    struct {
        bool home:1;
        bool save:1;
    };
    float tolerance;
    int repetitions;
    float max_adjust;

    // quad-gantry-leveling.temperatureswitch.hotend.switch
    uint16_t z0_switch_cs;
    uint16_t z1_switch_cs;
    uint16_t z2_switch_cs;
    uint16_t z3_switch_cs;
};

#endif

