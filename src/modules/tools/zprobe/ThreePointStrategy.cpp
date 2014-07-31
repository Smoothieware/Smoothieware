#include "ThreePointStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"

#include <string>
#include <algorithm>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")


bool ThreePointStrategy::handleConfig()
{
    for (int i = 0; i < 3; ++i) {
        std::string s= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
        if(s.empty()) break;
        // format is xxx,yyy


    }
    return true;
}

bool ThreePointStrategy::handleGcode(Gcode *gcode)
{
    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            return true;
        }

    } else if(gcode->has_m) {
        // handle mcodes
    }

    return false;
}


