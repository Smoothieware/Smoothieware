#include "DeltaCalibrationStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "BaseSolution.h"
#include "StepperMotor.h"

#include <cmath>
#include <tuple>
#include <algorithm>

#define radius_checksum         CHECKSUM("radius")
#define initial_height_checksum CHECKSUM("initial_height")

// deprecated
#define probe_radius_checksum CHECKSUM("probe_radius")

bool DeltaCalibrationStrategy::handleConfig()
{
    // default is probably wrong
    float r= THEKERNEL->config->value(leveling_strategy_checksum, delta_calibration_strategy_checksum, radius_checksum)->by_default(-1)->as_number();
    if(r == -1) {
        // deprecated config syntax]
        r =  THEKERNEL->config->value(zprobe_checksum, probe_radius_checksum)->by_default(100.0F)->as_number();
    }
    this->probe_radius= r;

    // the initial height above the bed we stop the intial move down after home to find the bed
    // this should be a height that is enough that the probe will not hit the bed and is the actual absolute z to move to before probing for the bed
    this->initial_height= THEKERNEL->config->value(leveling_strategy_checksum, delta_calibration_strategy_checksum, initial_height_checksum)->by_default(10)->as_number();
    return true;
}

bool DeltaCalibrationStrategy::handleGcode(Gcode *gcode)
{
    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 32 ) { // auto calibration for delta, Z bed mapping for cartesian
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

            // turn off any compensation transform as it will be invalidated anyway by this
            THEROBOT->compensationTransform= nullptr;

            if(!gcode->has_letter('R')) {
                if(!calibrate_delta_endstops(gcode)) {
                    gcode->stream->printf("Calibration failed to complete, check the initial probe height and/or initial_height settings\n");
                    return true;
                }
            }
            if(!gcode->has_letter('E')) {
                if(!calibrate_delta_radius(gcode)) {
                    gcode->stream->printf("Calibration failed to complete, check the initial probe height and/or initial_height settings\n");
                    return true;
                }
            }
            gcode->stream->printf("Calibration complete, save settings with M500\n");
            return true;

        }else if (gcode->g == 29) {
            // probe the 7 points
            if(!probe_delta_points(gcode)) {
                gcode->stream->printf("Calibration failed to complete, check the initial probe height and/or initial_height settings\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        // handle mcodes
    }

    return false;
}

// calculate the X and Y positions for the three towers given the radius from the center
static std::tuple<float, float, float, float, float, float> getCoordinates(float radius)
{
    float px = 0.866F * radius; // ~sin(60)
    float py = 0.5F * radius; // cos(60)
    float t1x = -px, t1y = -py; // X Tower
    float t2x = px, t2y = -py; // Y Tower
    float t3x = 0.0F, t3y = radius; // Z Tower
    return std::make_tuple(t1x, t1y, t2x, t2y, t3x, t3y);
}


// Probes the 7 points on a delta can be used for off board calibration
bool DeltaCalibrationStrategy::probe_delta_points(Gcode *gcode)
{
    float bedht= findBed();
    if(isnan(bedht)) return false;

    gcode->stream->printf("probe delta pints. initial Bed ht is %f mm\n", bedht);

    // check probe ht
    float mm;
    if(!zprobe->doProbeAt(mm, 0, 0)) return false;
    float dz = zprobe->getProbeHeight() - mm;
    gcode->stream->printf("center probe: %1.4f\n", dz);

    // get probe points
    float t1x, t1y, t2x, t2y, t3x, t3y;
    std::tie(t1x, t1y, t2x, t2y, t3x, t3y) = getCoordinates(this->probe_radius);

    // gather probe points
    float pp[][2] {{t1x, t1y}, {t2x, t2y}, {t3x, t3y}, {0, 0}, {-t1x, -t1y}, {-t2x, -t2y}, {-t3x, -t3y}};

    float max_delta= 0;
    float last_z= NAN;
    float start_z= THEROBOT->actuators[2]->get_current_position();

    for(auto& i : pp) {
        float mm;
        if(!zprobe->doProbeAt(mm, i[0], i[1])) return false;
        float z = mm;
        if(gcode->subcode == 0) {
            // prints the delta Z moved at the XY coordinates given
            gcode->stream->printf("X:%1.4f Y:%1.4f Z:%1.4f\n", i[0], i[1], z);

        }else if(gcode->subcode == 1) {
            // format that can be pasted here http://escher3d.com/pages/wizards/wizarddelta.php
            gcode->stream->printf("X%1.4f Y%1.4f Z%1.4f\n", i[0], i[1], start_z - z); // actual Z of bed at probe point
        }

        if(isnan(last_z)) {
            last_z= z;
        }else{
            max_delta= std::max(max_delta, fabsf(z-last_z));
        }
    }

    gcode->stream->printf("max delta: %f\n", max_delta);

    return true;
}

float DeltaCalibrationStrategy::findBed()
{
    // home
    zprobe->home();

    // move to an initial position fast so as to not take all day, we move down to initial_height, which is set in config, default 10mm
    float deltaz = initial_height;
    zprobe->coordinated_move(NAN, NAN, deltaz, zprobe->getFastFeedrate());
    zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate()); // move to 0,0

    // find bed, run at slow rate so as to not hit bed hard
    float mm;
    if(!zprobe->run_probe(mm, zprobe->getSlowFeedrate())) return NAN;

    // leave the probe zprobe->getProbeHeight() above bed
    float dz= zprobe->getProbeHeight();
    zprobe->coordinated_move(NAN, NAN, dz, zprobe->getSlowFeedrate(), true); // relative move

    return THEROBOT->get_axis_position(Z_AXIS); // Z position to move to from home to probe height above bed
}

/* Run a calibration routine for a delta
    1. Home
    2. probe for z bed
    3. probe initial tower positions
    4. set initial trims such that trims will be minimal negative values
    5. home, probe three towers again
    6. calculate trim offset and apply to all trims
    7. repeat 5, 6 until it converges on a solution
*/

bool DeltaCalibrationStrategy::calibrate_delta_endstops(Gcode *gcode)
{
    float target = 0.03F;
    if(gcode->has_letter('I')) target = gcode->get_value('I'); // override default target
    if(gcode->has_letter('J')) this->probe_radius = gcode->get_value('J'); // override default probe radius

    bool keep = false;
    if(gcode->has_letter('K')) keep = true; // keep current settings

    gcode->stream->printf("Calibrating Endstops: target %fmm, radius %fmm\n", target, this->probe_radius);

    // get probe points
    float t1x, t1y, t2x, t2y, t3x, t3y;
    std::tie(t1x, t1y, t2x, t2y, t3x, t3y) = getCoordinates(this->probe_radius);

    float trimx = 0.0F, trimy = 0.0F, trimz = 0.0F;
    if(!keep) {
        // zero trim values
        if(!set_trim(0, 0, 0, gcode->stream)) return false;

    } else {
        // get current trim, and continue from that
        if (get_trim(trimx, trimy, trimz)) {
            gcode->stream->printf("Current Trim X: %f, Y: %f, Z: %f\r\n", trimx, trimy, trimz);

        } else {
            gcode->stream->printf("Could not get current trim, are endstops enabled?\n");
            return false;
        }
    }

    // find the bed, as we potentially have a temporary z probe we don't know how low under the nozzle it is
    // so we need to find the initial place that the probe triggers when it hits the bed
    float bedht= findBed();
    if(isnan(bedht)) return false;
    gcode->stream->printf("initial Bed ht is %f mm\n", bedht);

    // check probe ht
    float mm;
    if(!zprobe->doProbeAt(mm, 0, 0)) return false;
    float dz = zprobe->getProbeHeight() - mm;
    gcode->stream->printf("center probe: %1.4f\n", dz);
    if(fabsf(dz) > target) {
         gcode->stream->printf("Probe was not repeatable to %f mm, (%f)\n", target, dz);
         return false;
    }

    // get initial probes
    // probe the base of the X tower
    if(!zprobe->doProbeAt(mm, t1x, t1y)) return false;
    float t1z = mm;
    gcode->stream->printf("T1-0 Z:%1.4f\n", t1z);

    // probe the base of the Y tower
    if(!zprobe->doProbeAt(mm, t2x, t2y)) return false;
    float t2z = mm;
    gcode->stream->printf("T2-0 Z:%1.4f\n", t2z);

    // probe the base of the Z tower
    if(!zprobe->doProbeAt(mm, t3x, t3y)) return false;
    float t3z = mm;
    gcode->stream->printf("T3-0 Z:%1.4f\n", t3z);

    float trimscale = 1.2522F; // empirically determined

    auto mmx = std::minmax({t1z, t2z, t3z});
    if((mmx.second - mmx.first) <= target) {
        gcode->stream->printf("trim already set within required parameters: delta %f\n", mmx.second - mmx.first);
        return true;
    }

    // set trims to worst case so we always have a negative trim
    trimx += (mmx.first - t1z) * trimscale;
    trimy += (mmx.first - t2z) * trimscale;
    trimz += (mmx.first - t3z) * trimscale;

    for (int i = 1; i <= 10; ++i) {
        // set trim
        if(!set_trim(trimx, trimy, trimz, gcode->stream)) return false;

        // home and move probe to start position just above the bed
        zprobe->home();
        zprobe->coordinated_move(NAN, NAN, bedht, zprobe->getFastFeedrate()); // do a move from home to the point above the bed

        // probe the base of the X tower
        if(!zprobe->doProbeAt(mm, t1x, t1y)) return false;
        t1z = mm;
        gcode->stream->printf("T1-%d Z:%1.4f\n", i, t1z);

        // probe the base of the Y tower
        if(!zprobe->doProbeAt(mm, t2x, t2y)) return false;
        t2z = mm;
        gcode->stream->printf("T2-%d Z:%1.4f\n", i, t2z);

        // probe the base of the Z tower
        if(!zprobe->doProbeAt(mm, t3x, t3y)) return false;
        t3z = mm;
        gcode->stream->printf("T3-%d Z:%1.4f\n", i, t3z);

        mmx = std::minmax({t1z, t2z, t3z});
        if((mmx.second - mmx.first) <= target) {
            gcode->stream->printf("trim set to within required parameters: delta %f\n", mmx.second - mmx.first);
            break;
        }

        // set new trim values based on min difference
        trimx += (mmx.first - t1z) * trimscale;
        trimy += (mmx.first - t2z) * trimscale;
        trimz += (mmx.first - t3z) * trimscale;

        // flush the output
        THEKERNEL->call_event(ON_IDLE);
    }

    if((mmx.second - mmx.first) > target) {
        gcode->stream->printf("WARNING: trim did not resolve to within required parameters: delta %f\n", mmx.second - mmx.first);
    }

    return true;
}

/*
    probe edges to get outer positions, then probe center
    modify the delta radius until center and X converge
*/

bool DeltaCalibrationStrategy::calibrate_delta_radius(Gcode *gcode)
{
    float target = 0.03F;
    if(gcode->has_letter('I')) target = gcode->get_value('I'); // override default target
    if(gcode->has_letter('J')) this->probe_radius = gcode->get_value('J'); // override default probe radius

    gcode->stream->printf("Calibrating delta radius: target %f, radius %f\n", target, this->probe_radius);

    // get probe points
    float t1x, t1y, t2x, t2y, t3x, t3y;
    std::tie(t1x, t1y, t2x, t2y, t3x, t3y) = getCoordinates(this->probe_radius);

    // find the bed, as we potentially have a temporary z probe we don't know how low under the nozzle it is
    // so we need to find thr initial place that the probe triggers when it hits the bed
    float bedht= findBed();
    if(isnan(bedht)) return false;
    gcode->stream->printf("initial Bed ht is %f mm\n", bedht);

    // check probe ht
    float mm;
    if(!zprobe->doProbeAt(mm, 0, 0)) return false;
    float dz = zprobe->getProbeHeight() - mm;
    gcode->stream->printf("center probe: %1.4f\n", dz);
    if(fabsf(dz) > target) {
         gcode->stream->printf("Probe was not repeatable to %f mm, (%f)\n", target, dz);
         return false;
    }

    // probe center to get reference point at this Z height
    float dc;
    if(!zprobe->doProbeAt(dc, 0, 0)) return false;
    gcode->stream->printf("CT Z:%1.3f\n", dc);
    float cmm = dc;

    // get current delta radius
    float delta_radius = 0.0F;
    BaseSolution::arm_options_t options;
    if(THEROBOT->arm_solution->get_optional(options)) {
        delta_radius = options['R'];
    }
    if(delta_radius == 0.0F) {
        gcode->stream->printf("This appears to not be a delta arm solution\n");
        return false;
    }
    options.clear();

    bool good= false;
    float drinc = 2.5F; // approx
    for (int i = 1; i <= 10; ++i) {
        // probe t1, t2, t3 and get average, but use coordinated moves, probing center won't change
        float dx, dy, dz;
        if(!zprobe->doProbeAt(dx, t1x, t1y)) return false;
        gcode->stream->printf("T1-%d Z:%1.3f\n", i, dx);
        if(!zprobe->doProbeAt(dy, t2x, t2y)) return false;
        gcode->stream->printf("T2-%d Z:%1.3f\n", i, dy);
        if(!zprobe->doProbeAt(dz, t3x, t3y)) return false;
        gcode->stream->printf("T3-%d Z:%1.3f\n", i, dz);

        // now look at the difference and reduce it by adjusting delta radius
        float m = (dx + dy + dz) / 3.0F;
        float d = cmm - m;
        gcode->stream->printf("C-%d Z-ave:%1.4f delta: %1.3f\n", i, m, d);

        if(fabsf(d) <= target){
            good= true;
            break; // resolution of success
        }

        // increase delta radius to adjust for low center
        // decrease delta radius to adjust for high center
        delta_radius += (d * drinc);

        // set the new delta radius
        options['R'] = delta_radius;
        THEROBOT->arm_solution->set_optional(options);
        gcode->stream->printf("Setting delta radius to: %1.4f\n", delta_radius);

        zprobe->home();
        zprobe->coordinated_move(NAN, NAN, bedht, zprobe->getFastFeedrate()); // move to absolute Z that is just above bed

        // flush the output
        THEKERNEL->call_event(ON_IDLE);
    }

    if(!good) {
        gcode->stream->printf("WARNING: delta radius did not resolve to within required parameters: %f\n", target);
    }

    return true;
}

bool DeltaCalibrationStrategy::set_trim(float x, float y, float z, StreamOutput *stream)
{
    float t[3] {x, y, z};
    bool ok = PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
        stream->printf("set trim to X:%f Y:%f Z:%f\n", x, y, z);
    } else {
        stream->printf("unable to set trim, is endstops enabled?\n");
    }

    return ok;
}

bool DeltaCalibrationStrategy::get_trim(float &x, float &y, float &z)
{
    void *returned_data;
    bool ok = PublicData::get_value( endstops_checksum, trim_checksum, &returned_data );

    if (ok) {
        float *trim = static_cast<float *>(returned_data);
        x = trim[0];
        y = trim[1];
        z = trim[2];
        return true;
    }
    return false;
}
