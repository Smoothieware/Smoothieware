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
#include "Plane3D.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")
#define home_checksum                CHECKSUM("home_first")
#define tolerance_checksum           CHECKSUM("tolerance")

ThreePointStrategy::ThreePointStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    for (int i = 0; i < 3; ++i) {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
    plane = nullptr;
}

ThreePointStrategy::~ThreePointStrategy()
{
    delete plane;
}

bool ThreePointStrategy::handleConfig()
{
    // format is xxx,yyy for the probe points
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    if(!p1.empty()) probe_points[0] = parseXY(p1.c_str());
    if(!p2.empty()) probe_points[1] = parseXY(p2.c_str());
    if(!p3.empty()) probe_points[2] = parseXY(p3.c_str());

    this->home= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, home_checksum)->by_default(true)->as_bool();
    this->tolerance= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    return true;
}

bool ThreePointStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        // G code processing
        if( gcode->g == 31 ) { // report status
            if(this->plane == nullptr) {
                 gcode->stream->printf("Bed leveling plane is not set\n");
            }else{
                 gcode->stream->printf("Bed leveling plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
            }
            gcode->stream->printf("Probe is %s\n", zprobe->getProbeStatus() ? "Triggered" : "Not triggered");
            return true;

        } else if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();
            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                gcode->stream->printf("Probe completed, bed plane defined\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        if(gcode->m == 557) { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2
            int idx = 0;
            float x = NAN, y = NAN;
            if(gcode->has_letter('P')) idx = gcode->get_value('P');
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(idx >= 0 && idx <= 2) {
                probe_points[idx] = std::make_tuple(x, y);
            }else{
                 gcode->stream->printf("only 3 probe points allowed P0-P2\n");
            }
            return true;

        } else if(gcode->m == 561) { // M561: Set Identity Transform
            delete this->plane;
            this->plane= nullptr;
            // TODO delete the adjustZfnc in robot
            return true;

        } else if(gcode->m == 503) {
            gcode->stream->printf(";Probe points:\n");
            for (int i = 0; i < 3; ++i) {
                float x, y;
                std::tie(x, y) = probe_points[i];
                gcode->stream->printf("M557 P%d X%1.5f Y%1.5f\n", i, x, y);
            }
            // TODO encode plane if set and M500
            return true;

        } else if(gcode->m == 999) {
            // DEBUG run a test M999 A B C X Y set Z to A B C and test for point at X Y
            Vector3 v[3];
            float x, y, z, a= 0, b= 0, c= 0;
            if(gcode->has_letter('A')) a = gcode->get_value('A');
            if(gcode->has_letter('B')) b = gcode->get_value('B');
            if(gcode->has_letter('C')) c = gcode->get_value('C');
            std::tie(x, y) = probe_points[0]; v[0].set(x, y, a);
            std::tie(x, y) = probe_points[1]; v[1].set(x, y, b);
            std::tie(x, y) = probe_points[2]; v[2].set(x, y, c);
            delete this->plane;
            this->plane = new Plane3D(v[0], v[1], v[2]);
            gcode->stream->printf("plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
            x= 0; y=0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            z= getZOffset(x, y);
            gcode->stream->printf("z= %f\n", z);
            return true;
        }
    }

    return false;
}

void ThreePointStrategy::homeXY()
{
    Gcode gc("G28 X0 Y0", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
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

    // optionally home XY axis first, but allow for manual homing
    if(this->home)
        homeXY();

    // move to the first probe point
    std::tie(x, y) = probe_points[0];
    zprobe->coordinated_move(x, y, NAN, zprobe->getFastFeedrate());

    // for now we use probe to find bed and not the Z min endstop
    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    int s;
    if(!zprobe->run_probe(s, true)) return false;
    // do we need to set set to Z == 0 here? as the rest is relative anyway

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    // probe the three points
    Vector3 v[3];
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        float z = zprobe->probeDistance(x, y);
        if(isnan(z)) return false; // probe failed
        z -= zprobe->getProbeHeight(); // relative distance between the probe points
        stream->printf("DEBUG: P%d:%1.4f\n", i, z);
        v[i].set(x, y, z);
    }

    // if first point is not within tolerance of probe height report it.
    if(abs(v[0][2] - zprobe->getProbeHeight()) > this->tolerance) {
        stream->printf("WARNING: probe is not within tolerance\n");
    }

    // define the plane
    delete this->plane;
    // check tolerance level here default 0.03mm
    auto mm = std::minmax({v[0][2], v[1][2], v[2][2]});
    if((mm.second - mm.first) <= this->tolerance) {
        this->plane= nullptr; // plane is flat no need to do anything
        stream->printf("DEBUG: flat plane\n");
        // THEKERNEL->robot->adjustZfnc= nullptr;
    }else{
        this->plane = new Plane3D(v[0], v[1], v[2]);
        stream->printf("DEBUG: plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
        // TODO set the adjustZfnc in robot
        // THEKERNEL->robot->adjustZfnc= [this](float x, float y) { return this->getZOffset(x, y); }
    }

    return true;
}

// find the Z offset for the point on the plane at x, y
float ThreePointStrategy::getZOffset(float x, float y)
{
    if(this->plane == nullptr) return NAN;
    return this->plane->getz(x, y);
}

// parse a "X,Y" string return x,y
std::tuple<float, float> ThreePointStrategy::parseXY(const char *str)
{
    float x = NAN, y = NAN;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, nullptr);
    }
    return std::make_tuple(x, y);
}
