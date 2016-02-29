// This code is derived from (and mostly copied from) Johann Rocholls code at https://github.com/jcrocholl/Marlin/blob/deltabot/Marlin/Marlin_main.cpp
// license is the same as his code.

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

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define grid_radius_checksum         CHECKSUM("radius")
#define grid_resolution_checksum     CHECKSUM("resolution")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_checksum                CHECKSUM("save")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define initial_height_checksum      CHECKSUM("initial_height")

DeltaGridStrategy::DeltaGridStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    // TODO allocate grid in AHB0 or AHB1
    //grid = nullptr;
    reset_bed_level();
}

DeltaGridStrategy::~DeltaGridStrategy()
{
    //delete[] grid;
}

bool DeltaGridStrategy::handleConfig()
{
    grid_radius = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, grid_radius_checksum)->by_default(50.0F)->as_number();
    //grid_resolution = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, grid_radius_checksum)->by_default(7)->as_number();
    tolerance = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    save = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, save_checksum)->by_default(false)->as_bool();
    // the initial height above the bed we stop the intial move down after home to find the bed
    // this should be a height that is enough that the probe will not hit the bed and is an offset from max_z (can be set to 0 if max_z takes into account the probe offset)
    this->initial_height= THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, initial_height_checksum)->by_default(10)->as_number();

    // Probe offsets xxx,yyy,zzz
    {
        std::string po = THEKERNEL->config->value(leveling_strategy_checksum, delta_grid_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
        std::vector<float> v = parse_number_list(po.c_str());
        if(v.size() >= 3) {
            this->probe_offsets = std::make_tuple(v[0], v[1], v[2]);
        }
    }

    return true;
}

void DeltaGridStrategy::save_grid(StreamOutput *stream)
{
    FILE *fp= fopen("/sd/delta.grid", "w");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid\n");
        return;
    }

    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
            if(fwrite(&grid[x][y], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to write grid\n");
                fclose(fp);
                return;
            }
        }
    }
    fclose(fp);
}

void DeltaGridStrategy::load_grid(StreamOutput *stream)
{
    FILE *fp= fopen("/sd/delta.grid", "r");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid\n");
        return;
    }

    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
            if(fread(&grid[x][y], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to read grid\n");
                fclose(fp);
                return;
            }
        }
    }
    fclose(fp);
}

bool DeltaGridStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if( gcode->g == 31 ) { // do probe (should be 32 but use 31 as deltacalibration will usually also be enabled)
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
            remove("delta.grid");
            return true;

        } else if(gcode->m == 374) { // M374: Save grid
            save_grid(gcode->stream);
            return true;

        } else if(gcode->m == 375) { // M375: load grid, M375.1 display grid
            if(gcode->subcode == 1) {
                print_bed_level(gcode->stream);
            }else{
                load_grid(gcode->stream);
                setAdjustFunction(true);
            }

        } else if(gcode->m == 565) { // M565: Set Z probe offsets
            float x= 0, y= 0, z= 0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            if(gcode->has_letter('Z')) z = gcode->get_value('Z');
            probe_offsets = std::make_tuple(x, y, z);
            return true;

        } else if(gcode->m == 500 || gcode->m == 503) { // M500 save, M503 display
            float x, y, z;
            gcode->stream->printf(";Probe offsets:\n");
            std::tie(x, y, z) = probe_offsets;
            gcode->stream->printf("M565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);
            if(save && gcode->m == 500) gcode->stream->printf(";Load default grid\nM375\n");
            return true;
        }
    }

    return false;
}

// set the rectangle in which to probe
#define DELTA_PROBABLE_RADIUS (grid_radius)
#define LEFT_PROBE_BED_POSITION (-DELTA_PROBABLE_RADIUS)
#define RIGHT_PROBE_BED_POSITION (DELTA_PROBABLE_RADIUS)
#define BACK_PROBE_BED_POSITION (DELTA_PROBABLE_RADIUS)
#define FRONT_PROBE_BED_POSITION (-DELTA_PROBABLE_RADIUS)

// probe at the points of a lattice grid
//#define AUTO_BED_LEVELING_GRID_POINTS (grid_resolution)
#define AUTO_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS - 1))
#define AUTO_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS - 1))

#define X_PROBE_OFFSET_FROM_EXTRUDER std::get<0>(probe_offsets)
#define Y_PROBE_OFFSET_FROM_EXTRUDER std::get<1>(probe_offsets)
#define Z_PROBE_OFFSET_FROM_EXTRUDER std::get<2>(probe_offsets)

void DeltaGridStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the compensationTransform in robot
        THEKERNEL->robot->compensationTransform= [this](float target[3]) { doCompensation(target); };
    }else{
        // clear it
        THEKERNEL->robot->compensationTransform= nullptr;
    }
}

float DeltaGridStrategy::findBed()
{
    // home
    zprobe->home();

    // move to an initial position fast so as to not take all day, we move down max_z - initial_height, which is set in config, default 10mm
    float deltaz= zprobe->getMaxZ() - initial_height;
    zprobe->coordinated_move(NAN, NAN, -deltaz, zprobe->getFastFeedrate(), true); // relative move
    zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate()); // move to 0,0

    // find bed at 0,0 run at slow rate so as to not hit bed hard
    int s;
    if(!zprobe->run_probe(s, false)) return NAN;

    return zprobe->zsteps_to_mm(s) + deltaz - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed
}

bool DeltaGridStrategy::doProbe(Gcode *gc)
{
    reset_bed_level();
    setAdjustFunction(false);

    float initial_z= findBed();
    if(isnan(initial_z)) return false;
    gc->stream->printf("initial Bed ht is %f mm\n", initial_z);

    // we start the probe zprobe->getProbeHeight() above the bed
    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, -initial_z, zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    float radius= DELTA_PROBABLE_RADIUS;
    if(gc->has_letter('J')) radius = gc->get_value('J'); // override default probe radius

    for (int yCount = 0; yCount < AUTO_BED_LEVELING_GRID_POINTS; yCount++) {
        float yProbe = FRONT_PROBE_BED_POSITION + AUTO_BED_LEVELING_GRID_Y * yCount;
        int xStart, xStop, xInc;
        if (yCount % 2) {
            xStart = 0;
            xStop = AUTO_BED_LEVELING_GRID_POINTS;
            xInc = 1;
        } else {
            xStart = AUTO_BED_LEVELING_GRID_POINTS - 1;
            xStop = -1;
            xInc = -1;
        }

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
            float xProbe = LEFT_PROBE_BED_POSITION + AUTO_BED_LEVELING_GRID_X * xCount;

            // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
            float distance_from_center = sqrtf(xProbe * xProbe + yProbe * yProbe);
            if (distance_from_center > radius) continue;

            int s;
            if(!zprobe->doProbeAt(s, xProbe-X_PROBE_OFFSET_FROM_EXTRUDER, yProbe-Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
            float measured_z = zprobe->getProbeHeight() - zprobe->zsteps_to_mm(s); // this is the delta z from bed at 0,0
            grid[xCount][yCount] = measured_z;
        }
    }

    extrapolate_unprobed_bed_level();
    print_bed_level(gc->stream);

    setAdjustFunction(true);

    return true;
}

void DeltaGridStrategy::extrapolate_one_point(int x, int y, int xdir, int ydir)
{
    if (!isnan(grid[x][y])) {
        return;  // Don't overwrite good values.
    }
    float a = 2 * grid[x + xdir][y] - grid[x + xdir * 2][y]; // Left to right.
    float b = 2 * grid[x][y + ydir] - grid[x][y + ydir * 2]; // Front to back.
    float c = 2 * grid[x + xdir][y + ydir] - grid[x + xdir * 2][y + ydir * 2]; // Diagonal.
    float median = c;  // Median is robust (ignores outliers).
    if (a < b) {
        if (b < c) median = b;
        if (c < a) median = a;
    } else {  // b <= a
        if (c < b) median = b;
        if (a < c) median = a;
    }
    grid[x][y] = median;
}

// Fill in the unprobed points (corners of circular print surface)
// using linear extrapolation, away from the center.
void DeltaGridStrategy::extrapolate_unprobed_bed_level()
{
    int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
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
    int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
    float grid_x = std::max(0.001F - half, std::min(half - 0.001F, target[X_AXIS] / AUTO_BED_LEVELING_GRID_X));
    float grid_y = std::max(0.001F - half, std::min(half - 0.001F, target[Y_AXIS] / AUTO_BED_LEVELING_GRID_Y));
    int floor_x = floorf(grid_x);
    int floor_y = floorf(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = grid[floor_x + half][floor_y + half];
    float z2 = grid[floor_x + half][floor_y + half + 1];
    float z3 = grid[floor_x + half + 1][floor_y + half];
    float z4 = grid[floor_x + half + 1][floor_y + half + 1];
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
    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
            stream->printf("%1.4f ", grid[x][y]);
        }
        stream->printf("\n");
    }
}

// Reset calibration results to zero.
void DeltaGridStrategy::reset_bed_level()
{
    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
            grid[x][y] = NAN;
        }
    }
}
