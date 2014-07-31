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
#include <cstdlib>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")


bool ThreePointStrategy::handleConfig()
{
    // format is xxx,yyy for the probe points
    std::string p1= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    if(!p1.empty()) probe_points[0]= parseXY(p1.c_str());
    if(!p2.empty()) probe_points[1]= parseXY(p2.c_str());
    if(!p3.empty()) probe_points[2]= parseXY(p3.c_str());
    return true;
}

bool ThreePointStrategy::handleGcode(Gcode *gcode)
{
    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();
            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered\n");
            }else{
                gcode->stream->printf("Probe completed, bed plane defined\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        if(gcode->m == 557) { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2
            int idx= 0;
            float x= NAN, y= NAN;
            if(gcode->has_letter('P')) idx= gcode->get_value('P');
            if(gcode->has_letter('X')) x= gcode->get_value('X');
            if(gcode->has_letter('Y')) y= gcode->get_value('Y');
            if(idx >= 0 && idx <= 2) {
                probe_points[idx]= std::make_tuple(x, y);
            }
            return true;

        }else if(gcode->m == 503) {
            gcode->stream->printf(";Probe points:\n");
            for (int i = 0; i < 3; ++i) {
                float x, y;
                std::tie(x, y) = probe_points[i];
                gcode->stream->printf("M557 P%d X%1.5f Y%1.5f\n", i, x, y);
            }
            return true;
        }
    }

    return false;
}

bool ThreePointStrategy::doProbing(StreamOutput *stream)
{
    float x, y;
    // check the probe points have been defined
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        if(isnan(x) || isnan(y)) {
             stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
             return false;
        }
    }

    // home presuming a cartesian homing to 0,0,0
    zprobe->home();

    // move to start position and probe from there
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    // probe the three points
    Vector3 v[3];
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        float z= zprobe->probeDistance(x, y) - zprobe->getProbeHeight(); // relative distance between the probe points
        if(isnan(z)) return false; // probe failed
        stream->printf("DEBUG: P%d:%1.4f\n", i, z);
        v[i].set(x, y, z);
    }

    // define the plane
    delete this->plane;
    this->plane= new Plane3D(v[0], v[1], v[2]);

    stream->printf("DEBUG: plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);

    return true;
}

// find the Z offset for the point on the plane at x, y
float ThreePointStrategy::getZOffset(float x, float y)
{
    if(plane == nullptr) return NAN;
    return plane->getz(x, y);
}

// parse a "X,Y" string return x,y
std::tuple<float, float> ThreePointStrategy::parseXY(const char *str){
    float x=NAN, y=NAN;
    char *p;
    x= strtof(str, &p);
    if(p+1 < str+strlen(str)) {
        y= strtof(p+1, nullptr);
    }
    return std::make_tuple(x, y);
}
