/*
    Based on ThreePointStrategy by Jim Morris (wolfmanjm@gmail.com)
    QGL code for Voron by Christopher Lang (christopher.lang@acurana.de)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes four user specified points on the bed and determines how much each of
    the four Z steppers need to be adjusted to lie exactly in one plane.

    Inspired by the QGL code from Klipper.

// calculate straight line functions for levels measured and then move all
// motors into same position

//# Leveling code for XY rails that are controlled by Z steppers as in:
//#
//# Z stepper1 ----> O                             O <---- Z stepper2
//#                  | * <-- probe1   probe2 --> * |
//#                  |                             |
//#                  |                             | <--- Y2 rail
//#   Y1 rail -----> |                             |
//#                  |                             |
//#                  |=============================|
//#                  |            ^                |
//#                  |            |                |
//#                  |   X rail --/                |
//#                  |                             |
//#                  | * <-- probe0   probe3 --> * |
//# Z stepper0 ----> O                             O <---- Z stepper3


//##  Gantry Corners for Voron 300mm Build
//#gantry_corners:
//#   -60,-10
//#   360,370
//##  Probe points
//#points:
//#   50,25
//#   50,225
//#   250,225
//#   250,25

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

    leveling-strategy.quad-gantry-leveling.enable         true

    Four probe points must be defined, these are best if they are the four points of a square, as far apart as possible.
    They can be defined in the config file as:

    leveling-strategy.quad-gantry-leveling.point1         50.0,25.0   # the first probe point (x,y)
    leveling-strategy.quad-gantry-leveling.point2         50.0,225.0  # the second probe point (x,y)
    leveling-strategy.quad-gantry-leveling.point3         250.0,225.0 # the third probe point (x,y)
    leveling-strategy.quad-gantry-leveling.point4         250.0,25.0  # the fourth probe point (x,y)

    or they may be defined (and saved with M500) using M557 P0 X30 Y40.5  where P is 0,1,2,3

    The gantry corners need to be defined as:

    leveling-strategy.quad-gantry-leveling.gantry_corner0 -60.0,4.5   # the gantry corner for probe point 0, used for QGL calculation
    leveling-strategy.quad-gantry-leveling.gantry_corner2 360.0,375.5 # the gantry corner for probe point 2, used for QGL

    These values are for a 300mm Voron 2.2, adjust for your Voron.

    Probe offsets from the nozzle or tool head can be defined with

    leveling-strategy.quad-gantry-leveling.probe_offsets  0,0,0  # probe offsets x,y,z

    They may also be set with M565 X0 Y0 Z0

    To force homing in X and Y before G32 does the probe the following can be set in config, this is the default

    leveling-strategy.quad-gantry-leveling.home_first    true   # disable by setting to false

    The probe tolerance can be set using the config line

    leveling-strategy.quad-gantry-leveling.tolerance   0.03    # the probe tolerance in mm, default is 0.03mm

    This QGL code assumes that the Z axis is driven by one step signal but that a switch (from the Switch module)
    is used to disable each of the steppers. This allows for independent adjustment of each motor.

    There is an example on how to use the Switch module to disable a motor in the
    Smoothieware documentation (->Switch Module). When using Open-Drain wiring, you need to add “o!” to your pin numbers.
    http://smoothieware.org/switch#homing-a-multi-motor-axis

    leveling-strategy.quad-gantry-leveling.max_adjust     10      # max. adjustment in mm for QGL before failing
    leveling-strategy.quad-gantry-leveling.repetitions    3       # number of distance measurements per point
    leveling-strategy.quad-gantry-leveling.switch_z       z-0     # the switch that is used to enable / disable the STEP pin of the motor
    leveling-strategy.quad-gantry-leveling.switch_z1      z-1
    leveling-strategy.quad-gantry-leveling.switch_z2      z-2
    leveling-strategy.quad-gantry-leveling.switch_z3      z-3

    There is a config sample in the ConfigSamples/Snippets directory.

    Usage
    -----
    G29 probes the four probe points and reports the Z at each point, if a plane is active it will be used to level the probe
    G32 probes the four probe points and adjusts the Z steppers, this will remain in effect until reset or motors are switched off
    G31 reports the status

    M557 defines the probe points
    M565 defines the probe offsets from the nozzle or tool head

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "QGLSwitchStrategy.h"
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
#include "utils.h"
#include "SwitchPublicAccess.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")
#define probe_point_4_checksum       CHECKSUM("point4")
#define repetitions_checksum         CHECKSUM("repetitions")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define home_checksum                CHECKSUM("home_first")
#define tolerance_checksum           CHECKSUM("tolerance")
#define switch_z0_checksum           CHECKSUM("switch_z")
#define switch_z1_checksum           CHECKSUM("switch_z1")
#define switch_z2_checksum           CHECKSUM("switch_z2")
#define switch_z3_checksum           CHECKSUM("switch_z3")
#define gantry_corner0_checksum      CHECKSUM("gantry_corner0")
#define gantry_corner2_checksum      CHECKSUM("gantry_corner2")
#define max_adjust_checksum          CHECKSUM("max_adjust")

#define QGL_NUM_OF_TURNS             16 // the number of rounds until HALT is asserted


/*
 * calculate and save the slope
 * of a straight line
 */
class slope {
public:
    slope() : m (0) { };
    slope(std::tuple<float, float> p1, std::tuple<float, float> p2) {
        m = (std::get<Y_>(p2) - std::get<Y_>(p1)) /
                (std::get<X_>(p2) - std::get<X_>(p1)); };

    ~slope() { };

    float get() { return m; }

private:
    float m;
};


/*
 * class representing a function for a
 * straight_line: y=m*x+t
 */
class line {
public:
    line() : t(0) { };

    line(std::tuple<float, float> p1, std::tuple<float, float> p2) {
        m = slope(p1, p2);
        t = std::get<Y_>(p2) - m.get() * std::get<X_>(p2); };

    line(slope s, std::tuple<float, float> p2) {
        m = s;
        t = std::get<Y_>(p2) - m.get() * std::get<X_>(p2); };

    ~line() { };

    float plot(float x) {
        return m.get() * x + t;
    };

    float get_slope() { return m.get(); };

private:
    slope m;
    float t;
};


QGLSwitchStrategy::QGLSwitchStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    for (int i = 0; i < 4; ++i) {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
}

QGLSwitchStrategy::~QGLSwitchStrategy()
{
}

bool QGLSwitchStrategy::handleConfig()
{
    // format is xxx,yyy for the probe points
    // default to Voron 300mm
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, probe_point_1_checksum)->by_default("50,25")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, probe_point_2_checksum)->by_default("50,225")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, probe_point_3_checksum)->by_default("250,225")->as_string();
    std::string p4 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, probe_point_4_checksum)->by_default("250,25")->as_string();
    if(!p1.empty()) probe_points[0] = parseXY(p1.c_str());
    if(!p2.empty()) probe_points[1] = parseXY(p2.c_str());
    if(!p3.empty()) probe_points[2] = parseXY(p3.c_str());
    if(!p4.empty()) probe_points[3] = parseXY(p4.c_str());

    // how many times to repeat the measurement per point
    this->repetitions = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, repetitions_checksum)->by_default(3)->as_int();
    if (this->repetitions <= 0)
        this->repetitions = 1; // at least one

    // Probe offsets xxx,yyy,zzz
    // default to Voron 300mm
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,25,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    this->home= THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, home_checksum)->by_default(true)->as_bool();
    this->tolerance= THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();

    // default to Voron 300mm
    std::string gc0 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, gantry_corner0_checksum)->by_default("-60.0,-10.0")->as_string();
    if(!gc0.empty()) gantry_corner0 = parseXY(gc0.c_str());

    std::string gc2 = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, gantry_corner2_checksum)->by_default("360.0,370.0")->as_string();
    if(!gc2.empty()) gantry_corner2 = parseXY(gc2.c_str());

    this->max_adjust = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, max_adjust_checksum)->by_default(10.0F)->as_number();

    // load settings from config file
    string switchname = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, switch_z0_checksum)->by_default("")->as_string();
    if(switchname.empty()) {
        // no switch specified so invalid entry
        THEKERNEL->streams->printf("WARNING QGLSwitchStrategy: no z switch specified\n");
        return nullptr;
    }
    this->z0_switch_cs = get_checksum(switchname); // checksum of the switch to use

    switchname = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, switch_z1_checksum)->by_default("")->as_string();
    if(switchname.empty()) {
        // no switch specified so invalid entry
        THEKERNEL->streams->printf("WARNING QGLSwitchStrategy: no z1 switch specified\n");
        return nullptr;
    }
    this->z1_switch_cs = get_checksum(switchname); // checksum of the switch to use

    switchname = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, switch_z2_checksum)->by_default("")->as_string();
    if(switchname.empty()) {
        // no switch specified so invalid entry
        THEKERNEL->streams->printf("WARNING QGLSwitchStrategy: no z2 switch specified\n");
        return nullptr;
    }
    this->z2_switch_cs = get_checksum(switchname); // checksum of the switch to use

    switchname = THEKERNEL->config->value(leveling_strategy_checksum, quad_gantry_leveling_strategy_checksum, switch_z3_checksum)->by_default("")->as_string();
    if(switchname.empty()) {
        // no switch specified so invalid entry
        THEKERNEL->streams->printf("WARNING QGLSwitchStrategy: no z3 switch specified\n");
        return nullptr;
    }
    this->z3_switch_cs = get_checksum(switchname); // checksum of the switch to use

    return true;
}

bool QGLSwitchStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        // G code processing
        if(gcode->g == 29) { // test probe points for level
            if(!test_probe_points(gcode)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            }
            return true;

        } else if( gcode->g == 31 ) { // report status

            gcode->stream->printf("Probe is %s\n", zprobe->getProbeStatus() ? "Triggered" : "Not triggered");
            return true;

        } else if( gcode->g == 32 ) { // four point probe with adjust
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

             // clear any existing compensation
            setAdjustFunction(false);

            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                gcode->stream->printf("Probe completed, quad gantry leveled\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        if(gcode->m == 557) { // M557 - set probe points eg M557 P0 X30 Y40.5  where P is 0,1,2,3
            int idx = 0;
            float x = NAN, y = NAN;
            if(gcode->has_letter('P')) idx = gcode->get_value('P');
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(idx >= 0 && idx <= 3) {
                probe_points[idx] = std::make_tuple(x, y);
            }else{
                 gcode->stream->printf("only 4 probe points allowed P0-P3\n");
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
            for (int i = 0; i < 4; ++i) {
                std::tie(x, y) = probe_points[i];
                gcode->stream->printf("M557 P%d X%1.5f Y%1.5f\n", i, x, y);
            }
            gcode->stream->printf(";Probe offsets:\n");
            std::tie(x, y, z) = probe_offsets;
            gcode->stream->printf("M565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);

            return true;

        }
        #if 0
         else if(gcode->m == 9999) {
            // DEBUG run a test M9999 A B C X Y set Z to A B C and test for point at X Y
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

void QGLSwitchStrategy::homeXY()
{
    Gcode gc(THEKERNEL->is_grbl_mode() ? "G28.2 X0 Y0": "G28 X0 Y0", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}


bool QGLSwitchStrategy::doProbing(StreamOutput *stream)
{
    float x, y;

    // check the probe points have been defined
    for (int i = 0; i < 4; ++i) {
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
    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    float mm;
    if(!zprobe->run_probe(mm, zprobe->getSlowFeedrate())) return false;

    // TODO if using probe then we probably need to set Z to 0 at first probe point, but take into account probe offset from head
    THEROBOT->reset_axis_position(std::get<Z_AXIS>(this->probe_offsets), Z_AXIS);

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getSlowFeedrate()); // move to probe start position

    // probe the four points
    Vector3 v[4];
    bool within_tolerance[4];

    int j;
    for (j=0; j<QGL_NUM_OF_TURNS; ++j) {

        for (int i = 0; i < 4; ++i) {

            float z, z_sum = 0;
            std::tie(x, y) = probe_points[i];

            for (int k=0; k<this->repetitions; ++k) {

                // offset moves by the probe XY offset, 1st probe per point
                if(!zprobe->doProbeAt(z, k==0 ? x-std::get<X_AXIS>(this->probe_offsets) : NAN, k==0 ? y-std::get<Y_AXIS>(this->probe_offsets) : NAN )) return false;

                z= zprobe->getProbeHeight() - z; // relative distance between the probe points, lower is negative z
                stream->printf("DEBUG: #%02d: P%d:%.4f\n", k, i, z);

                z_sum += z;
            }

            z = z_sum / this->repetitions;

            stream->printf("DEBUG: AVG: P%d:%.4f\n", i, z);

            v[i] = Vector3(x, y, z);
        }

        stream->printf("DEBUG: gantry0: %f, %f\n", std::get<X_>(gantry_corner0), std::get<Y_>(gantry_corner0) );
        stream->printf("DEBUG: gantry2: %f, %f\n", std::get<X_>(gantry_corner2), std::get<Y_>(gantry_corner2) );


        // Calculate line along X axis between probe point 0 and 3
        std::tuple<float, float> ppx0(std::get<X_AXIS>(probe_points[0]), v[0][2]); // v[0][2] = z-value at probe point 0
        std::tuple<float, float> ppx3(std::get<X_AXIS>(probe_points[3]), v[3][2]);
        line line_x_pp03(ppx0, ppx3);

        stream->printf("DEBUG: line_x_pp03: m=%f, t=%f\n", line_x_pp03.get_slope(), line_x_pp03.plot(0));

        // Calculate slope along X axis between probe point 1 and 2
        std::tuple<float, float> ppx1(std::get<X_AXIS>(probe_points[1]), v[1][2]);
        std::tuple<float, float> ppx2(std::get<X_AXIS>(probe_points[2]), v[2][2]);
        line line_x_pp12(ppx1, ppx2);

        stream->printf("DEBUG: line_x_pp12: m=%f, t=%f\n", line_x_pp12.get_slope(), line_x_pp12.plot(0));

        // Calculate gantry line along Y axis between stepper 0 and 1
        std::tuple<float, float> a1(std::get<Y_AXIS>(probe_points[0]), line_x_pp03.plot(std::get<X_>(gantry_corner0)));
        std::tuple<float, float> a2(std::get<Y_AXIS>(probe_points[1]), line_x_pp12.plot(std::get<X_>(gantry_corner0)));
        line line_y_s01(a1, a2);

        stream->printf("DEBUG: line_y_s01: m=%f, t=%f\n", line_y_s01.get_slope(), line_y_s01.plot(0));

        // Calculate gantry line along Y axis between stepper 2 and 3
        std::tuple<float, float> b1(std::get<Y_AXIS>(probe_points[3]), line_x_pp03.plot(std::get<X_>(gantry_corner2)));
        std::tuple<float, float> b2(std::get<Y_AXIS>(probe_points[2]), line_x_pp12.plot(std::get<X_>(gantry_corner2)));
        line line_y_s23(b1, b2);

        stream->printf("DEBUG: line_y_s23: m=%f, t=%f\n", line_y_s23.get_slope(), line_y_s23.plot(0));

        // Calculate z height of each stepper
        float z_height[4] = { 0,0,0,0 };
        z_height[0] = line_y_s01.plot(std::get<Y_>(gantry_corner0));
        z_height[1] = line_y_s01.plot(std::get<Y_>(gantry_corner2));
        z_height[2] = line_y_s23.plot(std::get<Y_>(gantry_corner2));
        z_height[3] = line_y_s23.plot(std::get<Y_>(gantry_corner0));

        // print all items for debugging, sum up for averaging
        float z_ave = 0;
        for (int i = 0; i < 4; ++i) {
            stream->printf("DEBUG: GC%d: %.4f\n", i, z_height[i]);
            z_ave += z_height[i];
        }
        z_ave /= 4;

        stream->printf("DEBUG: max_adjust: %.4f\n", max_adjust);

        float z_adjust[4];
        for (int i = 0; i < 4; ++i) {
            z_adjust[i] = z_height[i] - z_ave;

            if (fabsf(z_adjust[i]) > max_adjust) {
                THEKERNEL->streams->printf("QGL: required adjustment %.4f is greater"
                        "than max_adjust %.4f - HALT asserted - reset or M999 required\n", z_adjust[i], max_adjust);
                THEKERNEL->call_event(ON_HALT, nullptr);
                return false;
            }
        }

        // all motors off except z0
        set_switch(z0_switch_cs, true);
        set_switch(z1_switch_cs, false);
        set_switch(z2_switch_cs, false);
        set_switch(z3_switch_cs, false);

        // correct z0
        stream->printf("DEBUG: adjust z0: %.4f\n", z_adjust[0]);

        if(fabsf(z_adjust[0]) < this->tolerance) {
            stream->printf("DEBUG: correction not necessary, adjust < tol: %.4f\n", z_adjust[0]);
            within_tolerance[0] = true;
        }
        else {
            zprobe->coordinated_move(NAN, NAN, z_adjust[0], zprobe->getSlowFeedrate(), true); // move z0 to be equal to z_ave
            within_tolerance[0] = false;
        }

        // all motors off except z1
        set_switch(z0_switch_cs, false);
        set_switch(z1_switch_cs, true);

        // correct z1
        stream->printf("DEBUG: adjust z1: %.4f\n", z_adjust[1]);

        if(fabsf(z_adjust[1]) < this->tolerance) {
            stream->printf("DEBUG: correction not necessary, adjust < tol: %.4f\n", z_adjust[1]);
            within_tolerance[1] = true;
        }
        else {
            zprobe->coordinated_move(NAN, NAN, z_adjust[1], zprobe->getSlowFeedrate(), true); // move z1 to be equal to z_ave
            within_tolerance[1] = false;
        }

        // all motors off except z2
        set_switch(z1_switch_cs, false);
        set_switch(z2_switch_cs, true);

        // correct z2
        stream->printf("DEBUG: adjust z2: %.4f\n", z_adjust[2]);

        if(fabsf(z_adjust[2]) < this->tolerance) {
            stream->printf("DEBUG: correction not necessary, adjust < tol: %.4f\n", z_adjust[2]);
            within_tolerance[2] = true;
        }
        else {
            zprobe->coordinated_move(NAN, NAN, z_adjust[2], zprobe->getSlowFeedrate(), true); // move z2 to be equal to z_ave
            within_tolerance[2] = false;
        }

        // all motors off except z3
        set_switch(z2_switch_cs, false);
        set_switch(z3_switch_cs, true);

        // correct z3
        stream->printf("DEBUG: adjust z3: %.4f\n", z_adjust[3]);

        if(fabsf(z_adjust[3]) < this->tolerance) {
            stream->printf("DEBUG: correction not necessary, adjust < tol: %.4f\n", z_adjust[3]);
            within_tolerance[3] = true;
        }
        else {
            zprobe->coordinated_move(NAN, NAN, z_adjust[3], zprobe->getSlowFeedrate(), true); // move z3 to be equal to z_ave
            within_tolerance[3] = false;
        }

        // all motors on again
        set_switch(z0_switch_cs, true);
        set_switch(z1_switch_cs, true);
        set_switch(z2_switch_cs, true);

        int n = 0;
        for (int i=0; i<4; ++i) {
            if (within_tolerance[i])
                n+=1;
        }
        if (n == 4) {
            // all 4 points are within tolerance
            stream->printf("QGL: all done\n");
            break; // done
        }
        else {
            continue; // next iteration
        }
    }

    if (j >=QGL_NUM_OF_TURNS) { // too many probes not within tolerance
        THEKERNEL->streams->printf("QGL: unable to level gantry (not within tolerance) - HALT asserted - reset or M999 required\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
        return false;
    }

    return true;
}


// Turn the switch on (true) or off (false)
void QGLSwitchStrategy::set_switch(uint16_t switch_cs, bool switch_state)
{
    // get current switch state
    struct pad_switch pad;
    bool ok = PublicData::get_value(switch_checksum, switch_cs, 0, &pad);
    if (!ok) {
        THEKERNEL->streams->printf("// Failed to get switch state.\r\n");
        return;
    }

    if(pad.state == switch_state) return; // switch is already in the requested state

    ok = PublicData::set_value(switch_checksum, switch_cs, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("// Failed changing switch state.\r\n");
    }
}


// Probes the 4 points and reports heights
bool QGLSwitchStrategy::test_probe_points(Gcode *gcode)
{
    // check the probe points have been defined
    float max_delta= 0;
    float last_z= NAN;
    for (int i = 0; i < 4; ++i) {
        float x, y;
        std::tie(x, y) = probe_points[i];
        if(isnan(x) || isnan(y)) {
            gcode->stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
            return false;
        }

        float z;
        if(!zprobe->doProbeAt(z, x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets))) return false;

        gcode->stream->printf("X:%1.4f Y:%1.4f Z:%1.4f\n", x, y, z);

        if(isnan(last_z)) {
            last_z= z;
        }else{
            max_delta= std::max(max_delta, fabsf(z-last_z));
        }
    }

    gcode->stream->printf("max delta: %f\n", max_delta);

    return true;
}


void QGLSwitchStrategy::setAdjustFunction(bool on)
{
    if(on) {
        THEKERNEL->streams->printf("WARNING: do not setAdjustFunction to true for for QGLSwitchStrategy\n");
    }else{
        // clear it
        THEROBOT->compensationTransform= nullptr; // this is needed !!!!!!!!!!!!!!!!###############
    }
}


// parse a "X,Y" string return x,y
std::tuple<float, float> QGLSwitchStrategy::parseXY(const char *str)
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
std::tuple<float, float, float> QGLSwitchStrategy::parseXYZ(const char *str)
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

