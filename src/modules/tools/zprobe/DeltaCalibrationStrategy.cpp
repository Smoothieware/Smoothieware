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

#include <tuple>
#include <algorithm>

#define radius_checksum       CHECKSUM("radius")
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
    return true;
}

bool DeltaCalibrationStrategy::handleGcode(Gcode *gcode)
{
    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 32 ) { // auto calibration for delta, Z bed mapping for cartesian
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            if(!gcode->has_letter('R')) {
                if(!calibrate_delta_endstops(gcode)) {
                    gcode->stream->printf("Calibration failed to complete, probe not triggered\n");
                    return true;
                }
            }
            if(!gcode->has_letter('E')) {
                if(!calibrate_delta_radius(gcode)) {
                    gcode->stream->printf("Calibration failed to complete, probe not triggered\n");
                    return true;
                }
            }
            gcode->stream->printf("Calibration complete, save settings with M500\n");
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

    // home
    zprobe->home();

    // find bed, run at fast rate
    int s;
    if(!zprobe->run_probe(s, true)) return false;

    float bedht = zprobe->zsteps_to_mm(s) - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed
    gcode->stream->printf("Bed ht is %f mm\n", bedht);

    // move to start position
    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    // get initial probes
    // probe the base of the X tower
    if(!zprobe->doProbeAt(s, t1x, t1y)) return false;
    float t1z = zprobe->zsteps_to_mm(s);
    gcode->stream->printf("T1-0 Z:%1.4f C:%d\n", t1z, s);

    // probe the base of the Y tower
    if(!zprobe->doProbeAt(s, t2x, t2y)) return false;
    float t2z = zprobe->zsteps_to_mm(s);
    gcode->stream->printf("T2-0 Z:%1.4f C:%d\n", t2z, s);

    // probe the base of the Z tower
    if(!zprobe->doProbeAt(s, t3x, t3y)) return false;
    float t3z = zprobe->zsteps_to_mm(s);
    gcode->stream->printf("T3-0 Z:%1.4f C:%d\n", t3z, s);

    float trimscale = 1.2522F; // empirically determined

    auto mm = std::minmax({t1z, t2z, t3z});
    if((mm.second - mm.first) <= target) {
        gcode->stream->printf("trim already set within required parameters: delta %f\n", mm.second - mm.first);
        return true;
    }

    // set trims to worst case so we always have a negative trim
    trimx += (mm.first - t1z) * trimscale;
    trimy += (mm.first - t2z) * trimscale;
    trimz += (mm.first - t3z) * trimscale;

    for (int i = 1; i <= 10; ++i) {
        // set trim
        if(!set_trim(trimx, trimy, trimz, gcode->stream)) return false;

        // home and move probe to start position just above the bed
        zprobe->home();
        zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

        // probe the base of the X tower
        if(!zprobe->doProbeAt(s, t1x, t1y)) return false;
        t1z = zprobe->zsteps_to_mm(s);
        gcode->stream->printf("T1-%d Z:%1.4f C:%d\n", i, t1z, s);

        // probe the base of the Y tower
        if(!zprobe->doProbeAt(s, t2x, t2y)) return false;
        t2z = zprobe->zsteps_to_mm(s);
        gcode->stream->printf("T2-%d Z:%1.4f C:%d\n", i, t2z, s);

        // probe the base of the Z tower
        if(!zprobe->doProbeAt(s, t3x, t3y)) return false;
        t3z = zprobe->zsteps_to_mm(s);
        gcode->stream->printf("T3-%d Z:%1.4f C:%d\n", i, t3z, s);

        mm = std::minmax({t1z, t2z, t3z});
        if((mm.second - mm.first) <= target) {
            gcode->stream->printf("trim set to within required parameters: delta %f\n", mm.second - mm.first);
            break;
        }

        // set new trim values based on min difference
        trimx += (mm.first - t1z) * trimscale;
        trimy += (mm.first - t2z) * trimscale;
        trimz += (mm.first - t3z) * trimscale;

        // flush the output
        THEKERNEL->call_event(ON_IDLE);
    }

    if((mm.second - mm.first) > target) {
        gcode->stream->printf("WARNING: trim did not resolve to within required parameters: delta %f\n", mm.second - mm.first);
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

    zprobe->home();
    // find bed, then move to a point 5mm above it
    int s;
    if(!zprobe->run_probe(s, true)) return false;
    float bedht = zprobe->zsteps_to_mm(s) - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed
    gcode->stream->printf("Bed ht is %f mm\n", bedht);

    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    // probe center to get reference point at this Z height
    int dc;
    if(!zprobe->doProbeAt(dc, 0, 0)) return false;
    gcode->stream->printf("CT Z:%1.3f C:%d\n", zprobe->zsteps_to_mm(dc), dc);
    float cmm = zprobe->zsteps_to_mm(dc);

    // get current delta radius
    float delta_radius = 0.0F;
    BaseSolution::arm_options_t options;
    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        delta_radius = options['R'];
    }
    if(delta_radius == 0.0F) {
        gcode->stream->printf("This appears to not be a delta arm solution\n");
        return false;
    }
    options.clear();

    float drinc = 2.5F; // approx
    for (int i = 1; i <= 10; ++i) {
        // probe t1, t2, t3 and get average, but use coordinated moves, probing center won't change
        int dx, dy, dz;
        if(!zprobe->doProbeAt(dx, t1x, t1y)) return false;
        gcode->stream->printf("T1-%d Z:%1.3f C:%d\n", i, zprobe->zsteps_to_mm(dx), dx);
        if(!zprobe->doProbeAt(dy, t2x, t2y)) return false;
        gcode->stream->printf("T2-%d Z:%1.3f C:%d\n", i, zprobe->zsteps_to_mm(dy), dy);
        if(!zprobe->doProbeAt(dz, t3x, t3y)) return false;
        gcode->stream->printf("T3-%d Z:%1.3f C:%d\n", i, zprobe->zsteps_to_mm(dz), dz);

        // now look at the difference and reduce it by adjusting delta radius
        float m = zprobe->zsteps_to_mm((dx + dy + dz) / 3.0F);
        float d = cmm - m;
        gcode->stream->printf("C-%d Z-ave:%1.4f delta: %1.3f\n", i, m, d);

        if(abs(d) <= target) break; // resolution of success

        // increase delta radius to adjust for low center
        // decrease delta radius to adjust for high center
        delta_radius += (d * drinc);

        // set the new delta radius
        options['R'] = delta_radius;
        THEKERNEL->robot->arm_solution->set_optional(options);
        gcode->stream->printf("Setting delta radius to: %1.4f\n", delta_radius);

        zprobe->home();
        zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // needs to be a relative coordinated move

        // flush the output
        THEKERNEL->call_event(ON_IDLE);
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
