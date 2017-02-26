/*
 This code is derived from (and mostly copied from) Johann Rocholls code at https://github.com/jcrocholl/Marlin/blob/deltabot/Marlin/Marlin_main.cpp
 license is the same as his code.

    Summary
    -------
    Probes grid_size points in X and Y (total probes grid_size * grid_size) and stores the relative offsets from the 0,0 Z height
    When enabled every move will calculate the Z offset based on interpolating the height offset within the grids nearest 4 points.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

      leveling-strategy.rectangular-grid.enable         true

    The size of the grid can be set with...

      leveling-strategy.rectangular-grid.size        7

      this is the X and Y size of the grid, it must be an odd number, the default is 7 which is 49 probe points

    The width and length of the rectangle that is probed is set with...

      leveling-strategy.rectangular-grid.x_size       100
      leveling-strategy.rectangular-grid.y_size       90

   Optionally probe offsets from the nozzle or tool head can be defined with...

      leveling-strategy.rectangular-grid.probe_offsets  0,0,0  # probe offsetrs x,y,z

      they may also be set with M565 X0 Y0 Z0

    If the saved grid is to be loaded on boot then this must be set in the config...

      leveling-strategy.rectangular-grid.save        true

      Then when M500 is issued it will save M375 which will cause the grid to be loaded on boot. The default is to not autoload the grid on boot

    Optionally an initial_height can be set that tell the intial probe where to stop the fast decent before it probes, this should be around 5-10mm above the bed
      leveling-strategy.rectangular-grid.initial_height  10


    Usage
    -----
    G29 test probes a rectangle which defaults to the width and height, can be overidden with Xnnn and Ynnn

    G31 probes the grid and turns the compensation on, this will remain in effect until reset or M561/M370
        optional parameters {{Xn}} {{Yn}} sets the size for this rectangular probe, which gets saved with M375

    M370 clears the grid and turns off compensation
    M374 Save grid to /sd/cartesian.grid
    M374.1 delete /sd/cartesian.grid
    M375 Load the grid from /sd/cartesian.grid and enable compensation
    M375.1 display the current grid
    M561 clears the grid and turns off compensation
    M565 defines the probe offsets from the nozzle or tool head


    M500 saves the probe points
    M503 displays the current settings
*/

#include "CartGridStrategy.h"

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
#include <fastmath.h>

#define grid_size_checksum           CHECKSUM("size")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_checksum                CHECKSUM("save")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define initial_height_checksum      CHECKSUM("initial_height")
#define x_size_checksum              CHECKSUM("x_size")
#define y_size_checksum              CHECKSUM("y_size")
#define do_home_checksum             CHECKSUM("do_home")

#define GRIDFILE "/sd/cartesian.grid"

CartGridStrategy::CartGridStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    grid = nullptr;
}

CartGridStrategy::~CartGridStrategy()
{
    if(grid != nullptr) AHB0.dealloc(grid);
}

bool CartGridStrategy::handleConfig()
{
    grid_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, grid_size_checksum)->by_default(7)->as_number();
    tolerance = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    save = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, save_checksum)->by_default(false)->as_bool();
    do_home = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, do_home_checksum)->by_default(true)->as_bool();

    x_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, x_size_checksum)->by_default(0.0F)->as_number();
    y_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, y_size_checksum)->by_default(0.0F)->as_number();
    if (x_size == 0.0F || y_size == 0.0F) {
        THEKERNEL->streams->printf("Error: Invalid config, x_size and y_size must be defined\n");
        return false;
    }

    // the initial height above the bed we stop the intial move down after home to find the bed
    // this should be a height that is enough that the probe will not hit the bed and is an offset from max_z (can be set to 0 if max_z takes into account the probe offset)
    this->initial_height = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, initial_height_checksum)->by_default(10)->as_number();

    // Probe offsets xxx,yyy,zzz
    {
        std::string po = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
        std::vector<float> v = parse_number_list(po.c_str());
        if(v.size() >= 3) {
            this->probe_offsets = std::make_tuple(v[0], v[1], v[2]);
        }
    }

    // allocate in AHB0
    grid = (float *)AHB0.alloc(grid_size * grid_size * sizeof(float));

    if(grid == nullptr) {
        THEKERNEL->streams->printf("Error: Not enough memory\n");
        return false;
    }

    reset_bed_level();

    return true;
}

void CartGridStrategy::save_grid(StreamOutput *stream)
{
    if(isnan(grid[0])) {
        stream->printf("error:No grid to save\n");
        return;
    }

    FILE *fp = fopen(GRIDFILE, "w");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid file %s\n", GRIDFILE);
        return;
    }

    if(fwrite(&grid_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error:Failed to write grid size\n");
        fclose(fp);
        return;
    }

    if(fwrite(&x_size, sizeof(float), 1, fp) != 1)  {
        stream->printf("error:Failed to write x_size\n");
        fclose(fp);
        return;
    }

    if(fwrite(&y_size, sizeof(float), 1, fp) != 1)  {
        stream->printf("error:Failed to write y_size\n");
        fclose(fp);
        return;
    }

    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            if(fwrite(&grid[x + (grid_size * y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to write grid\n");
                fclose(fp);
                return;
            }
        }
    }
    stream->printf("grid saved to %s\n", GRIDFILE);
    fclose(fp);
}

bool CartGridStrategy::load_grid(StreamOutput *stream)
{
    FILE *fp = fopen(GRIDFILE, "r");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid %s\n", GRIDFILE);
        return false;
    }

    uint8_t size;
    float x, y;

    if(fread(&size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error:Failed to read grid size\n");
        fclose(fp);
        return false;
    }

    if(size != grid_size) {
        stream->printf("error:grid size is different read %d - config %d\n", size, grid_size);
        fclose(fp);
        return false;
    }

    if(fread(&x, sizeof(float), 1, fp) != 1) {
        stream->printf("error:Failed to read grid x size\n");
        fclose(fp);
        return false;
    }

    if(fread(&y, sizeof(float), 1, fp) != 1) {
        stream->printf("error:Failed to read grid y size\n");
        fclose(fp);
        return false;
    }

    if(x != x_size || y != y_size) {
        stream->printf("error:bed dimensions changed read (%f, %f) - config (%f,%f)\n", x, y, x_size, y_size);
        fclose(fp);
        return false;
    }

    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            if(fread(&grid[x + (grid_size * y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to read grid\n");
                fclose(fp);
                return false;
            }
        }
    }
    stream->printf("grid loaded, grid: (%f, %f), size: %d\n", x_size, y_size, grid_size);
    fclose(fp);
    return true;
}

bool CartGridStrategy::probe_grid(int n, float x_size, float y_size, StreamOutput *stream)
{
    if(n < 5) {
        stream->printf("Need at least a 5x5 grid to probe\n");
        return true;
    }

    float initial_z = findBed();
    if(isnan(initial_z)) return false;

    float x_step= x_size / n;
    float y_step= y_size / n;
    for (int c = 0; c < n; ++c) {
        float y = y_step * c;
        for (int r = 0; r < n; ++r) {
            float x = x_step * r;
            float z = 0.0F;
            float mm;
            if(!zprobe->doProbeAt(mm, x, y)) return false;
            z = zprobe->getProbeHeight() - mm;
            stream->printf("%8.4f ", z);
        }
        stream->printf("\n");
    }
    return true;
}

bool CartGridStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        if (gcode->g == 29) { // do a probe to test flatness
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

            int n = gcode->has_letter('I') ? gcode->get_value('I') : 0;
            float x = x_size, y = y_size;
            if(gcode->has_letter('X')) x = gcode->get_value('X'); // override default probe width
            if(gcode->has_letter('Y')) y = gcode->get_value('Y'); // override default probe length
            if(n == 0) n = 7;
            probe_grid(n, x, y, gcode->stream);

            return true;

        } else if( gcode->g == 31 || gcode->g == 32) { // do a grid probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_idle();

            if(!doProbe(gcode)) {
                gcode->stream->printf("Probe failed to complete, check the initial probe height and/or initial_height settings\n");
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
            gcode->stream->printf("grid cleared and disabled\n");
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
            return true;

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
            if(save) {
                if(!isnan(grid[0])) gcode->stream->printf(";Load saved grid\nM375\n");
                else if(gcode->m == 503) gcode->stream->printf(";WARNING No grid to save\n");
            }
            return true;
        }
    }

    return false;
}

// These are convenience defines to keep the code as close to the original as possible it also saves memory and flash
// set the rectangle in which to probe
#define LEFT_PROBE_BED_POSITION (0)
#define RIGHT_PROBE_BED_POSITION (x_size)
#define BACK_PROBE_BED_POSITION (y_size)
#define FRONT_PROBE_BED_POSITION (0)

// probe at the points of a lattice grid
#define AUTO_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (grid_size - 1))
#define AUTO_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (grid_size - 1))

#define X_PROBE_OFFSET_FROM_EXTRUDER std::get<0>(probe_offsets)
#define Y_PROBE_OFFSET_FROM_EXTRUDER std::get<1>(probe_offsets)
#define Z_PROBE_OFFSET_FROM_EXTRUDER std::get<2>(probe_offsets)

void CartGridStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the compensationTransform in robot
        using std::placeholders::_1;
        using std::placeholders::_2;
        THEROBOT->compensationTransform = std::bind(&CartGridStrategy::doCompensation, this, _1, _2); // [this](float *target, bool inverse) { doCompensation(target, inverse); };
    } else {
        // clear it
        THEROBOT->compensationTransform = nullptr;
    }
}

float CartGridStrategy::findBed()
{
    if (do_home) zprobe->home();
    // move to an initial position fast so as to not take all day, we move down max_z - initial_height, which is set in config, default 10mm
    float deltaz = initial_height;
    zprobe->coordinated_move(NAN, NAN, deltaz, zprobe->getFastFeedrate());
    zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate()); // move to 0,0

    // find bed at 0,0 run at slow rate so as to not hit bed hard
    float mm;
    if(!zprobe->run_probe_return(mm, zprobe->getSlowFeedrate())) return NAN;

    float dz = zprobe->getProbeHeight() - mm;
    zprobe->coordinated_move(NAN, NAN, dz, zprobe->getFastFeedrate(), true); // relative move

    return mm + deltaz - zprobe->getProbeHeight(); // distance to move from home to 5mm above bed
}

bool CartGridStrategy::doProbe(Gcode *gc)
{
    gc->stream->printf("Rectangular Grid Probe...\n");
    setAdjustFunction(false);
    reset_bed_level();

    if(gc->has_letter('X')) x_size = gc->get_value('X'); // override default probe width, will get saved
    if(gc->has_letter('Y')) y_size = gc->get_value('Y'); // override default probe length, will get saved

    // find bed, and leave probe probe height above bed
    float initial_z = findBed();
    if(isnan(initial_z)) {
        gc->stream->printf("Finding bed failed, check the maxz and initial height settings\n");
        return false;
    }

    gc->stream->printf("Probe start ht is %f mm, rectangular bed width %fmm, height %fmm, grid size is %dx%d\n", initial_z, x_size, y_size, grid_size, grid_size);

    // do first probe for 0,0
    float mm;
    if(!zprobe->doProbeAt(mm, -X_PROBE_OFFSET_FROM_EXTRUDER, -Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
    float z_reference = zprobe->getProbeHeight() - mm; // this should be zero
    gc->stream->printf("probe at 0,0 is %f mm\n", z_reference);

    // probe all the points of the grid
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

            if(!zprobe->doProbeAt(mm, xProbe - X_PROBE_OFFSET_FROM_EXTRUDER, yProbe - Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
            float measured_z = zprobe->getProbeHeight() - mm - z_reference; // this is the delta z from bed at 0,0
            gc->stream->printf("DEBUG: X%1.4f, Y%1.4f, Z%1.4f\n", xProbe, yProbe, measured_z);
            grid[xCount + (grid_size * yCount)] = measured_z;
        }
    }

    print_bed_level(gc->stream);

    setAdjustFunction(true);

    return true;
}

void CartGridStrategy::doCompensation(float *target, bool inverse)
{
    // Adjust print surface height by linear interpolation over the bed_level array.
    float grid_x = std::max(0.001F, target[X_AXIS] / AUTO_BED_LEVELING_GRID_X);
    float grid_y = std::max(0.001F, target[Y_AXIS] / AUTO_BED_LEVELING_GRID_Y);
    int floor_x = floorf(grid_x);
    int floor_y = floorf(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = grid[(floor_x) + ((floor_y) * grid_size)];
    float z2 = grid[(floor_x) + ((floor_y + 1) * grid_size)];
    float z3 = grid[(floor_x + 1) + ((floor_y) * grid_size)];
    float z4 = grid[(floor_x + 1) + ((floor_y + 1) * grid_size)];
    float left = (1 - ratio_y) * z1 + ratio_y * z2;
    float right = (1 - ratio_y) * z3 + ratio_y * z4;
    float offset = (1 - ratio_x) * left + ratio_x * right;

    if(inverse)
        target[Z_AXIS] -= offset;
    else
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
void CartGridStrategy::print_bed_level(StreamOutput *stream)
{
    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            stream->printf("%7.4f ", grid[x + (grid_size * y)]);
        }
        stream->printf("\n");
    }
}

// Reset calibration results to zero.
void CartGridStrategy::reset_bed_level()
{
    for (int y = 0; y < grid_size; y++) {
        for (int x = 0; x < grid_size; x++) {
            grid[x + (grid_size * y)] = NAN;
        }
    }
}
