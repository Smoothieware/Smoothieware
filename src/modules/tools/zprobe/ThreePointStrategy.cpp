/*
    Author: Jim Morris (wolfmanjm@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes three user specified points on the bed and determines the plane of the bed relative to the probe.
    as the head moves in X and Y it will adjust Z to keep the head tram with the bed.

    Configuration
    -------------
    The strategy must be enabled in the cofnig as well as zprobe.

    leveling-strategy.three-point-leveling.enable         true

    Three probe points must be defined, these are best if they are the three points of an equilateral triangle, as far apart as possible.
    They can be defined in the config file as:-

    leveling-strategy.three-point-leveling.point1         100.0,0.0   # the first probe point (x,y)
    leveling-strategy.three-point-leveling.point2         200.0,200.0 # the second probe point (x,y)
    leveling-strategy.three-point-leveling.point3         0.0,200.0   # the third probe point (x,y)

    or they may be defined (and saved with M500) using M557 P0 X30 Y40.5  where P is 0,1,2

    probe offsets from the nozzle or tool head can be defined with

    leveling-strategy.three-point-leveling.probe_offsets  0,0,0  # probe offsetrs x,y,z

    they may also be set with M565 X0 Y0 Z0

    To force homing in X and Y before G32 does the probe the following can be set in config, this is the default

    leveling-strategy.three-point-leveling.home_first    true   # disable by setting to false

    The probe tolerance can be set using the config line

    leveling-strategy.three-point-leveling.tolerance   0.03    # the probe tolerance in mm, default is 0.03mm


    Usage
    -----
    G32 probes the three probe points and defines the bed plane, this will remain in effect until reset or M561
    G31 reports the status

    M557 defines the probe points
    M561 clears the plane and the bed leveling is disabled until G32 is run again
    M565 defines the probe offsets from the nozzle or tool head

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

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
#include "nuts_bolts.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define home_checksum                CHECKSUM("home_first")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_plane_checksum          CHECKSUM("save_plane")

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

    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    this->home= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, home_checksum)->by_default(true)->as_bool();
    this->tolerance= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    this->save= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, save_plane_checksum)->by_default(false)->as_bool();
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

        } else if(gcode->m == 561) { // M561: Set Identity Transform with no parameters, set the saved plane if A B C D are given
            delete this->plane;
            if(gcode->get_num_args() == 0) {
                this->plane= nullptr;
                // delete the compensationTransform in robot
                setAdjustFunction(false);
            }else{
                // smoothie specific way to restire a saved plane
                uint32_t a,b,c,d;
                a=b=c=d= 0;
                if(gcode->has_letter('A')) a = gcode->get_uint('A');
                if(gcode->has_letter('B')) b = gcode->get_uint('B');
                if(gcode->has_letter('C')) c = gcode->get_uint('C');
                if(gcode->has_letter('D')) d = gcode->get_uint('D');
                this->plane= new Plane3D(a, b, c, d);
                setAdjustFunction(true);
            }
            return true;

        } else if(gcode->m == 565) { // M565: Set Z probe offsets
            float x= 0, y= 0, z= 0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(gcode->has_letter('Z')) z = gcode->get_value('Z');
            probe_offsets = std::make_tuple(x, y, z);
            return true;

        } else if(gcode->m == 500 || gcode->m == 503) { // M500 save, M503 display
            float x, y, z;
            gcode->stream->printf(";Probe points:\n");
            for (int i = 0; i < 3; ++i) {
                std::tie(x, y) = probe_points[i];
                gcode->stream->printf("M557 P%d X%1.5f Y%1.5f\n", i, x, y);
            }
            gcode->stream->printf(";Probe offsets:\n");
            std::tie(x, y, z) = probe_offsets;
            gcode->stream->printf("M565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);

            // encode plane and save if set and M500 and enabled
            if(this->save && this->plane != nullptr) {
                if(gcode->m == 500) {
                    uint32_t a, b, c, d;
                    this->plane->encode(a, b, c, d);
                    gcode->stream->printf(";Saved bed plane:\nM561 A%lu B%lu C%lu D%lu \n", a, b, c, d);
                }else{
                    gcode->stream->printf(";The bed plane will be saved on M500\n");
                }
            }
            return true;

        }
        #if 0
         else if(gcode->m == 999) {
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
            // tell robot to adjust z on each move
            setAdjustFunction(true);
            return true;
        }
        #endif
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
    // offset by the probe XY offset
    x -= std::get<X_AXIS>(this->probe_offsets);
    y -= std::get<Y_AXIS>(this->probe_offsets);
    zprobe->coordinated_move(x, y, NAN, zprobe->getFastFeedrate());

    // for now we use probe to find bed and not the Z min endstop
    // the first probe point becomes Z == 0 effectively so if we home Z or manually set z after this, it needs to be at the first probe point

    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    int s;
    if(!zprobe->run_probe(s)) return false;

    // TODO if using probe then we probably need to set Z to 0 at first probe point, but take into account probe offset from head
    THEKERNEL->robot->reset_axis_position(std::get<Z_AXIS>(this->probe_offsets), Z_AXIS);

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getSlowFeedrate()); // move to probe start position

    // probe the three points
    Vector3 v[3];
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        // offset moves by the probe XY offset
        float z = zprobe->probeDistance(x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets));
        if(isnan(z)) return false; // probe failed
        z= zprobe->getProbeHeight() - z; // relative distance between the probe points, lower is negative z
        stream->printf("DEBUG: P%d:%1.4f\n", i, z);
        v[i].set(x, y, z);
    }

    // if first point is not within tolerance report it, it should ideally be 0
    if(abs(v[0][2]) > this->tolerance) {
        stream->printf("WARNING: probe is not within tolerance: %f > %f\n", abs(v[0][2]), this->tolerance);
    }

    // define the plane
    delete this->plane;
    // check tolerance level here default 0.03mm
    auto mm = std::minmax({v[0][2], v[1][2], v[2][2]});
    if((mm.second - mm.first) <= this->tolerance) {
        this->plane= nullptr; // plane is flat no need to do anything
        stream->printf("DEBUG: flat plane\n");
        // clear the compensationTransform in robot
        setAdjustFunction(false);

    }else{
        this->plane = new Plane3D(v[0], v[1], v[2]);
        stream->printf("DEBUG: plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
        setAdjustFunction(true);
    }

    return true;
}

void ThreePointStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the compensationTransform in robot
        THEKERNEL->robot->compensationTransform= [this](float target[3]) { target[2] += this->plane->getz(target[0], target[1]); };
    }else{
        // clear it
        THEKERNEL->robot->compensationTransform= nullptr;
    }
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

// parse a "X,Y,Z" string return x,y,z tuple
std::tuple<float, float, float> ThreePointStrategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z= 0;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, &p);
        if(p + 1 < str + strlen(str)) {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}
