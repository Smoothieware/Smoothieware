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
    or
      leveling-strategy.rectangular-grid.grid_x_size        7
      leveling-strategy.rectangular-grid.grid_y_size        7
    this is the X and Y size of the grid, it must be an odd number, the default is 7 which is 49 probe points

    If both "size" and "grid_x_size" and "grid_x_size defined "grid_x_size" and "grid_x_size" will be used.
    If "grid_x_size" and "grid_x_size" omitted then "size" will be used.
    If "size" omitted default value will be used.

    I and J params used for grid size. If both omitted values from config will be used. If only one provided (I or J) then it will be used for both x_size and y-size.

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

    If two corners rectangular mode activated using "leveling-strategy.rectangular-grid.only_by_two_corners true" then G29/31/32 will not work without providing XYAB parameters
        XY - start point, AB rectangle size from starting point
        "Two corners"" not absolutely correct name for this mode, because it use only one corner and rectangle size.

    Display mode of current grid can be changed to human redable mode (table with coordinates) by using
       leveling-strategy.rectangular-grid.human_readable  true

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
#define grid_x_size_checksum         CHECKSUM("grid_x_size")
#define grid_y_size_checksum         CHECKSUM("grid_y_size")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_checksum                CHECKSUM("save")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define initial_height_checksum      CHECKSUM("initial_height")
#define x_size_checksum              CHECKSUM("x_size")
#define y_size_checksum              CHECKSUM("y_size")
#define do_home_checksum             CHECKSUM("do_home")
#define m_attach_checksum            CHECKSUM("m_attach")
#define mount_position_checksum      CHECKSUM("mount_position")
#define only_by_two_corners_checksum CHECKSUM("only_by_two_corners")
#define human_readable_checksum      CHECKSUM("human_readable")
#define height_limit_checksum      CHECKSUM("height_limit")
#define dampening_start_checksum      CHECKSUM("dampening_start")

#define GRIDFILE "/sd/cartesian.grid"
#define GRIDFILE_NM "/sd/cartesian_nm.grid"

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

    uint8_t grid_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, grid_size_checksum)->by_default(7)->as_number();
    this->current_grid_x_size = this->configured_grid_x_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, grid_x_size_checksum)->by_default(grid_size)->as_number();
    this->current_grid_y_size = this->configured_grid_y_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, grid_y_size_checksum)->by_default(grid_size)->as_number();
    tolerance = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    save = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, save_checksum)->by_default(false)->as_bool();
    do_home = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, do_home_checksum)->by_default(true)->as_bool();
    only_by_two_corners = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, only_by_two_corners_checksum)->by_default(false)->as_bool();
    human_readable = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, human_readable_checksum)->by_default(false)->as_bool();
    do_manual_attach = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, m_attach_checksum)->by_default(false)->as_bool();

    this->height_limit = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, height_limit_checksum)->by_default(NAN)->as_number();
    this->dampening_start = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, dampening_start_checksum)->by_default(NAN)->as_number();

    if(!isnan(this->height_limit) && !isnan(this->dampening_start)) {
        this->damping_interval = height_limit - dampening_start;
    } else {
        this->damping_interval = NAN;
    }

    this->x_start = 0.0F;
    this->y_start = 0.0F;
    this->x_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, x_size_checksum)->by_default(0.0F)->as_number();
    this->y_size = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, y_size_checksum)->by_default(0.0F)->as_number();
    if (this->x_size == 0.0F || this->y_size == 0.0F) {
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

    //  manual attachment point xxx,yyy,zzz
    if (do_manual_attach)
    {
        std::string ap = THEKERNEL->config->value(leveling_strategy_checksum, cart_grid_leveling_strategy_checksum, mount_position_checksum)->by_default("0,0,50")->as_string();
        std::vector<float> w = parse_number_list(ap.c_str());
        if(w.size() >= 3) {
            this->m_attach = std::make_tuple(w[0], w[1], w[2]);
        }
    }

    // allocate in AHB0
    grid = (float *)AHB0.alloc(configured_grid_x_size * configured_grid_y_size * sizeof(float));

    if(grid == nullptr) {
        THEKERNEL->streams->printf("Error: Not enough memory\n");
        return false;
    }

    reset_bed_level();

    return true;
}

void CartGridStrategy::save_grid(StreamOutput *stream)
{
    if(only_by_two_corners){
        stream->printf("error:Unable to save grid in only_by_two_corners mode\n");
        return;
    }

    if(isnan(grid[0])) {
        stream->printf("error:No grid to save\n");
        return;
    }

    if((current_grid_x_size != configured_grid_x_size) || (current_grid_y_size != configured_grid_y_size)) {
        stream->printf("error:Unable to save grid with size different from configured\n");
        return;
    }

    FILE *fp = (configured_grid_x_size == configured_grid_y_size)?fopen(GRIDFILE, "w"):fopen(GRIDFILE_NM, "w");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid file %s\n", GRIDFILE);
        return;
    }
    uint8_t tmp_configured_grid_size = configured_grid_x_size;
    if(fwrite(&tmp_configured_grid_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error:Failed to write grid x size\n");
        fclose(fp);
        return;
    }

    tmp_configured_grid_size = configured_grid_y_size;
    if(configured_grid_y_size != configured_grid_x_size){
        if(fwrite(&tmp_configured_grid_size, sizeof(uint8_t), 1, fp) != 1) {
            stream->printf("error:Failed to write grid y size\n");
            fclose(fp);
            return;
        }
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

    for (int y = 0; y < configured_grid_y_size; y++) {
        for (int x = 0; x < configured_grid_x_size; x++) {
            if(fwrite(&grid[x + (configured_grid_x_size * y)], sizeof(float), 1, fp) != 1) {
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
    if(only_by_two_corners){
        stream->printf("error:Unable to load grid in only_by_two_corners mode\n");
        return false;
    }

    FILE *fp = (configured_grid_x_size == configured_grid_y_size)?fopen(GRIDFILE, "r"):fopen(GRIDFILE_NM, "r");
    if(fp == NULL) {
        stream->printf("error:Failed to open grid %s\n", GRIDFILE);
        return false;
    }

    uint8_t load_grid_x_size, load_grid_y_size;
    float x, y;

    if(fread(&load_grid_x_size, sizeof(uint8_t), 1, fp) != 1) {
        stream->printf("error:Failed to read grid size\n");
        fclose(fp);
        return false;
    }

    if(load_grid_x_size != configured_grid_x_size) {
        stream->printf("error:grid size x is different read %d - config %d\n", load_grid_x_size, configured_grid_x_size);
        fclose(fp);
        return false;
    }

    load_grid_y_size = load_grid_x_size;

    if(configured_grid_x_size != configured_grid_y_size){
        if(fread(&load_grid_y_size, sizeof(uint8_t), 1, fp) != 1) {
            stream->printf("error:Failed to read grid size\n");
            fclose(fp);
            return false;
        }

        if(load_grid_y_size != configured_grid_y_size) {
            stream->printf("error:grid size y is different read %d - config %d\n", load_grid_y_size, configured_grid_x_size);
            fclose(fp);
            return false;
        }
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

    for (int y = 0; y < configured_grid_y_size; y++) {
        for (int x = 0; x < configured_grid_x_size; x++) {
            if(fread(&grid[x + (configured_grid_x_size * y)], sizeof(float), 1, fp) != 1) {
                stream->printf("error:Failed to read grid\n");
                fclose(fp);
                return false;
            }
        }
    }
    stream->printf("grid loaded, grid: (%f, %f), size: %d x %d\n", x_size, y_size, load_grid_x_size, load_grid_y_size);
    fclose(fp);
    return true;
}

bool CartGridStrategy::probe_grid(int n, int m, float _x_start, float _y_start, float _x_size, float _y_size, StreamOutput *stream)
{
    if((n < 5)||(m < 5)) {
        stream->printf("Need at least a 5x5 grid to probe\n");
        return true;
    }


    if(!findBed()) return false;

    float x_step = _x_size / n;
    float y_step = _y_size / m;
    for (int c = 0; c < m; ++c) {
        float y = _y_start + y_step * c;
        for (int r = 0; r < n; ++r) {
            float x = _x_start + x_step * r;
            float z = 0.0F;
            float mm;
            if(!zprobe->doProbeAt(mm, x, y)) return false;
            z = zprobe->getProbeHeight() - mm;
            stream->printf("%1.4f ", z);
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

            int n = gcode->has_letter('I') ? gcode->get_value('I') : configured_grid_x_size;
            int m = gcode->has_letter('J') ? gcode->get_value('J') : configured_grid_y_size;

            float _x_size = this->x_size, _y_size = this->y_size;
            float _x_start = this->x_start, _y_start = this->y_start;

            if(only_by_two_corners){
                if(gcode->has_letter('X') && gcode->has_letter('Y') && gcode->has_letter('A') && gcode->has_letter('B')){
                    _x_start = gcode->get_value('X'); // override default probe start point
                    _y_start = gcode->get_value('Y'); // override default probe start point
                    _x_size = gcode->get_value('A'); // override default probe width
                    _y_size = gcode->get_value('B'); // override default probe length
                } else {
                    gcode->stream->printf("In only_by_two_corners mode all XYAB parameters needed\n");
                    return true;
                }
            } else {
                if(gcode->has_letter('X')) _x_size = gcode->get_value('X'); // override default probe width
                if(gcode->has_letter('Y')) _y_size = gcode->get_value('Y'); // override default probe length
            }

            probe_grid(n, m, _x_start, _y_start, _x_size, _y_size, gcode->stream);

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

bool CartGridStrategy::findBed()
{
    if (do_home && !do_manual_attach) zprobe->home();
    float z = initial_height;
    zprobe->coordinated_move(NAN, NAN, z, zprobe->getFastFeedrate()); // move Z only to initial_height
    zprobe->coordinated_move(x_start - X_PROBE_OFFSET_FROM_EXTRUDER, y_start - Y_PROBE_OFFSET_FROM_EXTRUDER, NAN, zprobe->getFastFeedrate()); // move at initial_height to x_start, y_start

    // find bed at 0,0 run at slow rate so as to not hit bed hard
    float mm;
    if(!zprobe->run_probe_return(mm, zprobe->getSlowFeedrate())) return false;

    // leave head probe_height above bed
    float dz = zprobe->getProbeHeight() - mm;
    zprobe->coordinated_move(NAN, NAN, dz, zprobe->getFastFeedrate(), true); // relative move

    return true;
}

bool CartGridStrategy::doProbe(Gcode *gc)
{
    gc->stream->printf("Rectangular Grid Probe...\n");

    if(only_by_two_corners){
        if(gc->has_letter('X') && gc->has_letter('Y') && gc->has_letter('A') && gc->has_letter('B')){
            this->x_start = gc->get_value('X'); // override default probe start point, will get saved
            this->y_start = gc->get_value('Y'); // override default probe start point, will get saved
            this->x_size = gc->get_value('A'); // override default probe width, will get saved
            this->y_size = gc->get_value('B'); // override default probe length, will get saved
        } else {
            gc->stream->printf("In only_by_two_corners mode all XYAB parameters needed\n");
            return false;
        }
    } else {
        if(gc->has_letter('X')) this->x_size = gc->get_value('X'); // override default probe width, will get saved
        if(gc->has_letter('Y')) this->y_size = gc->get_value('Y'); // override default probe length, will get saved
    }

    setAdjustFunction(false);
    reset_bed_level();

    if(gc->has_letter('I')) current_grid_x_size = gc->get_value('I'); // override default grid x size
    if(gc->has_letter('J')) current_grid_y_size = gc->get_value('J'); // override default grid y size

    if((this->current_grid_x_size * this->current_grid_y_size)  > (this->configured_grid_x_size * this->configured_grid_y_size)){
        gc->stream->printf("Grid size (%d x %d = %d) bigger than configured (%d x %d = %d). Change configuration.\n",
                            this->current_grid_x_size, this->current_grid_y_size, this->current_grid_x_size*this->current_grid_x_size,
                            this->configured_grid_x_size, this->configured_grid_y_size, this->configured_grid_x_size*this->configured_grid_y_size);
        return false;
    }

    if (do_manual_attach) {
        // Move to the attachment point defined
        if (do_home) zprobe->home();

        float x, y, z;
        std::tie(x, y, z) = m_attach;
        zprobe->coordinated_move( x, y, z , zprobe->getFastFeedrate());

        gc->stream->printf(" ************************************************************\n");
        gc->stream->printf("     Ensure probe is attached and trigger probe when done\n");
        gc->stream->printf(" ************************************************************\n");

        while( !zprobe->getProbeStatus()) {
            if(THEKERNEL->is_halted()) return(false);
            THEKERNEL->call_event(ON_IDLE);
        }
    }

    // find bed, and leave probe probe height above bed
    if(!findBed()) {
        gc->stream->printf("Finding bed failed, check the initial height setting\n");
        return false;
    }

    gc->stream->printf("Probe start ht is %f mm, rectangular bed width %fmm, height %fmm, grid size is %dx%d\n", zprobe->getProbeHeight(), x_size, y_size, current_grid_x_size, current_grid_y_size);

    // do first probe for 0,0
    float mm;
    if(!zprobe->doProbeAt(mm, this->x_start - X_PROBE_OFFSET_FROM_EXTRUDER, this->y_start - Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
    float z_reference = zprobe->getProbeHeight() - mm; // this should be zero
    gc->stream->printf("probe at 0,0 is %f mm\n", z_reference);

    // probe all the points of the grid
    for (int yCount = 0; yCount < this->current_grid_y_size; yCount++) {
        float yProbe = this->y_start + (this->y_size / (this->current_grid_y_size - 1)) * yCount;
        int xStart, xStop, xInc;
        if (yCount % 2) {
            xStart = this->current_grid_x_size - 1;
            xStop = -1;
            xInc = -1;
        } else {
            xStart = 0;
            xStop = this->current_grid_x_size;
            xInc = 1;
        }

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
            float xProbe = this->x_start + (this->x_size / (this->current_grid_x_size - 1)) * xCount;

            if(!zprobe->doProbeAt(mm, xProbe - X_PROBE_OFFSET_FROM_EXTRUDER, yProbe - Y_PROBE_OFFSET_FROM_EXTRUDER)) return false;
            float measured_z = zprobe->getProbeHeight() - mm - z_reference; // this is the delta z from bed at 0,0
            gc->stream->printf("DEBUG: X%1.4f, Y%1.4f, Z%1.4f\n", xProbe, yProbe, measured_z);
            grid[xCount + (this->current_grid_x_size * yCount)] = measured_z;
        }
    }

    print_bed_level(gc->stream);

    if (do_manual_attach) {
        // Move to the attachment point defined for removal of probe

        float x, y, z;
        std::tie(x, y, z) = m_attach;
        zprobe->coordinated_move( x, y, z , zprobe->getFastFeedrate());

        gc->stream->printf(" ********************\n");
        gc->stream->printf("     Remove probe\n");
        gc->stream->printf(" ********************\n");
    }

    setAdjustFunction(true);

    return true;
}

void CartGridStrategy::doCompensation(float *target, bool inverse)
{
    // Adjust print surface height by linear interpolation over the bed_level array.
    // offset scale: 1 for default (use offset as is)
    float scale = 1.0;
    if (!isnan(this->damping_interval)) {
        // if the height is below our compensation limit:
        if(target[Z_AXIS] <= this->height_limit) {
            // scale the offset as necessary:
            if(target[Z_AXIS] >= this->dampening_start) {
                scale = (1.0 - ((target[Z_AXIS] - this->dampening_start) / this->damping_interval));
            } // else leave scale at 1.0;
        } else {
            return; // if Z is higher than max, no compensation
        }
    }

    // find min/maxes, and handle the case where size is negative (assuming this is possible? Legacy code supported this)
    float min_x = std::min(this->x_start, this->x_start + this->x_size);
    float max_x = std::max(this->x_start, this->x_start + this->x_size);
    float min_y = std::min(this->y_start, this->y_start + this->y_size);
    float max_y = std::max(this->y_start, this->y_start + this->y_size);

    // clamp the input to the bounds of the compensation grid
    // if a point is beyond the bounds of the grid, it will get the offset of the closest grid point
    float x_target = std::min(std::max(target[X_AXIS], min_x), max_x);
    float y_target = std::min(std::max(target[Y_AXIS], min_y), max_y);

    float grid_x = std::max(0.001F, (x_target - this->x_start) / (this->x_size / (this->current_grid_x_size - 1)));
    float grid_y = std::max(0.001F, (y_target - this->y_start) / (this->y_size / (this->current_grid_y_size - 1)));
    int floor_x = floorf(grid_x);
    int floor_y = floorf(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = grid[(floor_x) + ((floor_y) * this->current_grid_x_size)];
    float z2 = grid[(floor_x) + ((floor_y + 1) * this->current_grid_x_size)];
    float z3 = grid[(floor_x + 1) + ((floor_y) * this->current_grid_x_size)];
    float z4 = grid[(floor_x + 1) + ((floor_y + 1) * this->current_grid_x_size)];
    float left = (1 - ratio_y) * z1 + ratio_y * z2;
    float right = (1 - ratio_y) * z3 + ratio_y * z4;
    float offset = (1 - ratio_x) * left + ratio_x * right;

    if (inverse) {
        target[Z_AXIS] -= offset * scale;
    } else {
        target[Z_AXIS] += offset * scale;
    }

    /*THEKERNEL->streams->printf("//DEBUG: TARGET: %f, %f, %f\n", target[0], target[1], target[2]);
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
     THEKERNEL->streams->printf("//DEBUG: scale= %f\n", scale);
     */
}


// Print calibration results for plotting or manual frame adjustment.
void CartGridStrategy::print_bed_level(StreamOutput *stream)
{
    if(!human_readable){
        for (int y = 0; y < current_grid_y_size; y++) {
            for (int x = 0; x < current_grid_x_size; x++) {
                stream->printf("%1.4f ", grid[x + (current_grid_x_size * y)]);
            }
            stream->printf("\n");
        }
    } else {

        int xStart = (x_size>0) ? 0 : (current_grid_x_size - 1);
        int xStop = (x_size>0) ? current_grid_x_size : -1;
        int xInc = (x_size>0) ? 1: -1;

        int yStart = (y_size<0) ? 0 : (current_grid_y_size - 1);
        int yStop = (y_size<0) ? current_grid_y_size : -1;
        int yInc = (y_size<0) ? 1: -1;

        for (int y = yStart; y != yStop; y += yInc) {
            stream->printf("%10.4f|", y * (y_size / (current_grid_y_size - 1)));
            for (int x = xStart; x != xStop; x += xInc) {
                stream->printf("%10.4f ",  grid[x + (current_grid_x_size * y)]);
            }
            stream->printf("\n");
        }
        stream->printf("           ");
        for (int x = xStart; x != xStop; x += xInc) {
            stream->printf("-----+-----");
        }
        stream->printf("\n");
        stream->printf("           ");
        for (int x = xStart; x != xStop; x += xInc) {
            stream->printf("%1.4f ",  x * (x_size / (current_grid_x_size - 1)));
        }
            stream->printf("\n");

    }

}

// Reset calibration results to zero.
void CartGridStrategy::reset_bed_level()
{
    for (int y = 0; y < current_grid_y_size; y++) {
        for (int x = 0; x < current_grid_x_size; x++) {
            grid[x + (current_grid_x_size * y)] = NAN;
        }
    }
}
