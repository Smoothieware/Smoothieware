/*
 This code is derived from (and mostly copied from) Johann Rocholls code at https://github.com/jcrocholl/Marlin/blob/deltabot/Marlin/Marlin_main.cpp
 license is the same as his code.

    Summary
    -------
    Probes grid_size points in X and Y (total probes grid_size * grid_size) and stores the relative offsets from the 0,0 Z height
    When enabled everymove will calcualte the Z offset based on interpolating the height offset within the grids nearest 4 points.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

      leveling-strategy.delta-grid.enable         true

    The radius of the bed must be specified with...

      leveling-strategy.delta-grid.radius        50

      this needs to be at least as big as the maximum printing radius as moves outside of this will not be compensated for correctly

    The size of the grid can be set with...

      leveling-strategy.delta-grid.size        7

      this is the X and Y size of the grid, it must be an odd number, the default is 7 which is 49 probe points

   Optionally probe offsets from the nozzle or tool head can be defined with...

      leveling-strategy.delta-grid.probe_offsets  0,0,0  # probe offsetrs x,y,z

      they may also be set with M565 X0 Y0 Z0

    If the saved grid is to be loaded on boot then this must be set in the config...

      leveling-strategy.delta-grid.save        true

      Then when M500 is issued it will save M375 which will cause the grid to be loaded on boot. The default is to not autoload the grid on boot

    Optionally an initial_height can be set that tell the intial probe where to stop the fast decent before it probes, this should be around 5-10mm above the bed
      leveling-strategy.delta-grid.initial_height  10


    Usage
    -----
    G29 test probes in a spiral pattern within the radius producing a map of offsets, this can be imported into a graphing program to visualize the bed heights
    G31 probes the grid and turns the compensation on, this will remain in effect until reset or M561/M370

    M370 clears the grid and turns off compensation
    M374 Save grid to /sd/delta.grid
    M374.1 delete /sd/delta.grid
    M375 Load the grid from /sd/delta.grid and enable compensation
    M375.1 display the current grid
    M561 clears the grid and turns off compensation
    M565 defines the probe offsets from the nozzle or tool head


    M500 saves the probe points
    M503 displays the current settings
*/

#include "DeltaGridStrategy.h"

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
#include "nuts_bolts.h"
#include "utils.h"
#include "platform_memory.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define grid_radius_checksum         CHECKSUM("radius")
#define grid_size_checksum           CHECKSUM("size")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_checksum                CHECKSUM("save")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define initial_height_checksum      CHECKSUM("initial_height")

#define GRIDFILE "/sd/delta.grid"

DeltaGridStrategy::DeltaGridStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    grid= nullptr;
}

DeltaGridStrategy::~DeltaGridStrategy()
{
    if(grid != nullptr) AHB0.dealloc(grid);
}

bool DeltaGridStrategy::handleConfig()
{
    grid_radius = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, grid_radius_checksum)->by_default(50.0F)->as_number();
    grid_size = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, grid_size_checksum)->by_default(7)->as_number();
    tolerance = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    save = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, save_checksum)->by_default(false)->as_bool();

    // the initial height above the bed we stop the intial move down after home to find the bed
    // this should be a height that is enough that the probe will not hit the bed and is an offset from max_z (can be set to 0 if max_z takes into account the probe offset)
    this->initial_height = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, initial_height_checksum)->by_default(10)->as_number();

    // Probe offsets xxx,yyy,zzz
    {
        std::string po = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
        std::vector<float> v = parse_number_list(po.c_str());
        if(v.size() >= 3) {
            this->probe_offsets = std::make_tuple(v[0], v[1], v[2]);
        }
    }

    // allocate in AHB0
    grid= (float *)AHB0.alloc(grid_size * grid_size * sizeof(float));

    reset_bed_level();

    return true;
}

void DeltaGridStrategy::save_grid(StreamOutput *stream)
{
    FILE *fp = fopen(GRIDFILE, "w");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid\n");
        return;
    }

    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            if(fwrite(&grid[x + (grid_size*y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to write grid\n");
                fclose(fp);
                return;
            }
        }
    }
    stream->printf("grid saved to %s\n", GRIDFILE);
    fclose(fp);
}

bool DeltaGridStrategy::load_grid(StreamOutput *stream)
{
    FILE *fp = fopen(GRIDFILE, "r");
    if(fp == NULL) {
        if(stream != nullptr) stream->printf("error:Failed to open grid\n");
        return false;
    }

    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            if(fread(&grid[x + (grid_size*y)], sizeof(float), 1, fp) != 1) {
                if(stream != nullptr) stream->printf("error:Failed to read grid\n");
                fclose(fp);
                return false;
            }
        }
    }
    if(stream != nullptr) stream->printf("grid loaded from %s\n", GRIDFILE);
    fclose(fp);
    return true;
}

// taken from Oskars PR #713
bool DeltaGridStrategy::probe_spiral(int n, StreamOutput *stream)
{
    float a = grid_radius / (2 * sqrtf(n * M_PI));
    float step_length = grid_radius * grid_radius / (2 * a * n);

    float initial_z = findBed();
    if(isnan(initial_z)) return false;

    auto theta = [a](float length) {return sqrtf(2*length/a); };

    float maxz= NAN, minz= NAN;
    for (int i = 0; i < n; i++) {
        float angle = theta(i * step_length);
        float r = angle * a;
        // polar to cartesian
        float x = r * cosf(angle);
        float y = r * sinf(angle);

        int steps;
        if (!zprobe->doProbeAt(steps, x, y)) return false;
        float z = zprobe->getProbeHeight() - zprobe->zsteps_to_mm(steps);
        stream->printf("PROBE: X%1.4f, Y%1.4f, Z%1.4f\n", x, y, z);
        if(isnan(maxz) || z > maxz) maxz= z;
        if(isnan(minz) || z < minz) minz= z;
    }

    stream->printf("max: %1.4f, min: %1.4f, delta: %1.4f\n", maxz, minz, maxz-minz);
    return true;
}

bool DeltaGridStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if (gcode->g == 29) { // do a spiral probe query
            int n= gcode->has_letter('I') ? gcode->get_value('I') : 50;
            probe_spiral(n, gcode->stream);
            return true;

        } else if( gcode->g == 31 ) { // do a grid probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            if(!doProbe(gcode)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                gcode->stream->printf("Probe completed\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        if(gcode->m == 370 || gcode->m == 561) { // M370: Clear bed, M561: Set Identity Transform
            // delete the compensationTransform in robot
            setAdjustFunction(false);
            reset_bed_level();
            return true;

        } else if(gcode->m == 374) { // M374: Save grid, M374.1: delete saved grid
            if(gcode->subcode == 1) {
                remove(GRIDFILE);
                gcode->stream->printf("%s deleted\n", GRIDFILE);
            } else {
                save_grid(gcode->stream);
            }

            return true;

        } else if(gcode->m == 375) { // M375: load grid, M375.1 display grid
            if(gcode->subcode == 1) {
                print_bed_level(gcode->stream);
            } else {
                if(load_grid(gcode->stream)) setAdjustFunction(true);
            }

        } else if(gcode->m == 565) { // M565: Set Z probe offsets
            float x = 0, y = 0, z = 0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(gcode->has_letter('Z')) z = gcode->get_value('Z');
            probe_offsets = std::make_tuple(x, y, z);
            return true;

        } else if(gcode->m == 500 || gcode->m == 503) { // M500 save, M503 display
            float x, y, z;
            std::tie(x, y, z) = probe_offsets;
            gcode->stream->printf(";Probe offsets:\nM565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);
            if(save && !isnan(grid[0])) gcode->stream->printf(";Load saved grid\nM375\n");
            return true;
        }
    }

    return false;
}

// These are convenience defines to keep the code as close to the original as possible it also saves memory and flash
// set the rectangle in which to probe
#define LEFT_PROBE_BED_POSITION (-grid_radius)
#define RIGHT_PROBE_BED_POSITION (grid_radius)
#define BACK_PROBE_BED_POSITION (grid_radius)
#define FRONT_PROBE_BED_POSITION (-grid_radius)

// probe at the points of a lattice grid
#define AUTO_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (grid_size - 1))
#define AUTO_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (grid_size - 1))

#define X_PROBE_OFFSET_FROM_EXTRUDER std::get<0>(probe_offsets)
#define Y_PROBE_OFFSET_FROM_EXTRUDER std::get<1>(probe_offsets)
#define Z_PROBE_OFFSET_FROM_EXTRUDER std::get<2>(probe_offsets)

void DeltaGridStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the compensationTransform in robot
        THEKERNEL->robot->compensationTransform = [this](float target[3]) { doCompensation(target); };
    } else {
        // clear it
        THEKERNEL->robot->compensationTransform = nullptr;
    }
}

float DeltaGridStrategy::findBed()
{
    // home
    zprobe->home();

    // move to an initial position fast so as to not take all day, we move down max_z - initial_height, which is set in config, default 10mm
    float deltaz = zprobe->getMaxZ() - initial_height;
    zprobe->coordinated_move(NAN, NAN, -deltaz, zprobe->getFastFeedrate(), true); // relative move
    zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate()); // move to 0,0

    // find bed at 0,0 run at slow rate so as to not hit bed hard
    int s;
    if(!zprobe->run_probe(s, false)) return NAN;

    // leave the probe zprobe->getProbeHeight() above bed
    zprobe->return_probe(s);
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight()-zprobe->zsteps_to_mm(s), zprobe->getFastFeedrate(), true); // relative move

    return zprobe->zsteps_to_mm(s) + deltaz - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed
}

bool DeltaGridStrategy::doProbe(Gcode *gc)
{
    reset_bed_level();
    setAdjustFunction(false);

    float initial_z = findBed();
    if(isnan(initial_z)) return false;
    gc->stream->printf("initial Bed ht is %f mm\n", initial_z);

    // do first probe for 0,0
    int s;
    if(!zprobe->doProbeAt(s, -X_PROBE_OFFSET_FROM_EXTRUDER, -Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
    float z_reference = zprobe->getProbeHeight() - zprobe->zsteps_to_mm(s); // this should be zero
    gc->stream->printf("probe at 0,0 is %f mm\n", z_reference);

    float radius = grid_radius;
    if(gc->has_letter('J')) radius = gc->get_value('J'); // override default probe radius

    for (int yCount = 0; yCount < grid_size; yCount++) {
        float yProbe = FRONT_PROBE_BED_POSITION + AUTO_BED_LEVELING_GRID_Y * yCount;
        int xStart, xStop, xInc;
        if (yCount % 2) {
            xStart = 0;
            xStop = grid_size;
            xInc = 1;
        } else {
            xStart = grid_size - 1;
            xStop = -1;
            xInc = -1;
        }

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
            float xProbe = LEFT_PROBE_BED_POSITION + AUTO_BED_LEVELING_GRID_X * xCount;

            // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
            float distance_from_center = sqrtf(xProbe * xProbe + yProbe * yProbe);
            if (distance_from_center > radius) continue;

            if(!zprobe->doProbeAt(s, xProbe - X_PROBE_OFFSET_FROM_EXTRUDER, yProbe - Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
            float measured_z = zprobe->getProbeHeight() - zprobe->zsteps_to_mm(s) - z_reference; // this is the delta z from bed at 0,0
            gc->stream->printf("DEBUG: X%1.4f, Y%1.4f, Z%1.4f\n", xProbe - X_PROBE_OFFSET_FROM_EXTRUDER, yProbe - Y_PROBE_OFFSET_FROM_EXTRUDER, measured_z);
            grid[xCount + (grid_size * yCount)] = measured_z;
        }
    }

    extrapolate_unprobed_bed_level();
    print_bed_level(gc->stream);

    setAdjustFunction(true);

    return true;
}

void DeltaGridStrategy::extrapolate_one_point(int x, int y, int xdir, int ydir)
{
    if (!isnan(grid[x + (grid_size*y)])) {
        return;  // Don't overwrite good values.
    }
    float a = 2 * grid[(x + xdir) + (y*grid_size)] - grid[(x + xdir * 2) + (y*grid_size)]; // Left to right.
    float b = 2 * grid[x + ((y + ydir) * grid_size)] - grid[x + ((y + ydir * 2) * grid_size)]; // Front to back.
    float c = 2 * grid[(x + xdir) + ((y + ydir) * grid_size)] - grid[(x + xdir * 2) + ((y + ydir * 2) * grid_size)]; // Diagonal.
    float median = c;  // Median is robust (ignores outliers).
    if (a < b) {
        if (b < c) median = b;
        if (c < a) median = a;
    } else {  // b <= a
        if (c < b) median = b;
        if (a < c) median = a;
    }
    grid[x + (grid_size*y)] = median;
}

// Fill in the unprobed points (corners of circular print surface)
// using linear extrapolation, away from the center.
void DeltaGridStrategy::extrapolate_unprobed_bed_level()
{
    int half = (grid_size - 1) / 2;
    for (int y = 0; y <= half; y++) {
        for (int x = 0; x <= half; x++) {
            if (x + y < 3) continue;
            extrapolate_one_point(half - x, half - y, x > 1 ? +1 : 0, y > 1 ? +1 : 0);
            extrapolate_one_point(half + x, half - y, x > 1 ? -1 : 0, y > 1 ? +1 : 0);
            extrapolate_one_point(half - x, half + y, x > 1 ? +1 : 0, y > 1 ? -1 : 0);
            extrapolate_one_point(half + x, half + y, x > 1 ? -1 : 0, y > 1 ? -1 : 0);
        }
    }
}

void DeltaGridStrategy::doCompensation(float target[3])
{
    // Adjust print surface height by linear interpolation over the bed_level array.
    int half = (grid_size - 1) / 2;
    float grid_x = std::max(0.001F - half, std::min(half - 0.001F, target[X_AXIS] / AUTO_BED_LEVELING_GRID_X));
    float grid_y = std::max(0.001F - half, std::min(half - 0.001F, target[Y_AXIS] / AUTO_BED_LEVELING_GRID_Y));
    int floor_x = floorf(grid_x);
    int floor_y = floorf(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = grid[(floor_x + half) + ((floor_y + half) * grid_size)];
    float z2 = grid[(floor_x + half) + ((floor_y + half + 1) * grid_size)];
    float z3 = grid[(floor_x + half + 1) + ((floor_y + half) * grid_size)];
    float z4 = grid[(floor_x + half + 1) + ((floor_y + half + 1) * grid_size)];
    float left = (1 - ratio_y) * z1 + ratio_y * z2;
    float right = (1 - ratio_y) * z3 + ratio_y * z4;
    float offset = (1 - ratio_x) * left + ratio_x * right;

    target[Z_AXIS] += offset;


    /*
        THEKERNEL->streams->printf("//DEBUG: TARGET: %f, %f, %f\n", target[0], target[1], target[2]);
        THEKERNEL->streams->printf("//DEBUG: grid_x= %f\n", grid_x);
        THEKERNEL->streams->printf("//DEBUG: grid_y= %f\n", grid_y);
        THEKERNEL->streams->printf("//DEBUG: floor_x= %d\n", floor_x);
        THEKERNEL->streams->printf("//DEBUG: floor_y= %d\n", floor_y);
        THEKERNEL->streams->printf("//DEBUG: ratio_x= %f\n", ratio_x);
        THEKERNEL->streams->printf("//DEBUG: ratio_y= %f\n", ratio_y);
        THEKERNEL->streams->printf("//DEBUG: z1= %f\n", z1);
        THEKERNEL->streams->printf("//DEBUG: z2= %f\n", z2);
        THEKERNEL->streams->printf("//DEBUG: z3= %f\n", z3);
        THEKERNEL->streams->printf("//DEBUG: z4= %f\n", z4);
        THEKERNEL->streams->printf("//DEBUG: left= %f\n", left);
        THEKERNEL->streams->printf("//DEBUG: right= %f\n", right);
        THEKERNEL->streams->printf("//DEBUG: offset= %f\n", offset);
    */
}


// Print calibration results for plotting or manual frame adjustment.
void DeltaGridStrategy::print_bed_level(StreamOutput *stream)
{
    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            stream->printf("%1.4f ", grid[x + (grid_size*y)]);
        }
        stream->printf("\n");
    }
}

// Reset calibration results to zero.
void DeltaGridStrategy::reset_bed_level()
{
    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            grid[x + (grid_size*y)] = NAN;
        }
    }
}
