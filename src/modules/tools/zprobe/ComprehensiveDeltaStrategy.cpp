/*

    Comprehensive Delta Strategy by 626Pilot
    This code requires a Z-probe. You can use your own, or get mine here: http://www.thingiverse.com/626Pilot/designs

    This strategy implements functionality found in the following separate strategies:
        * DeltaCalibrationStrategy (calibrate DR & endstops on delta printers) - my version is faster :)
        * ThreePointStrategy (adjust surface normal) - this is calibrated by simulated annealing (see below)
        * ZGridStrategy (depth-map print surface & adjust effector's Z according to that)

    This strategy ADDS the following functionality, which the other strategies lack:
        * Probe calibration
        * Parallel simulated annealing, a "weak AI" method of calibrating delta printers (all 14 variables, SIMULTANEOUSLY!!!)
        * Surface normal (like ThreePointStrategy) AND depth-mapped interpolation (like ZGridStrategy) at the same time
        * You don't have to pick whether you want one feature or another; you can use everything you need
        * Method Prefixes: All printed output gets prepended with a method prefix, so you ALWAYS know which method printed anything
        *                  ~ Call push_prefix("XX") to specify the current method's two-character prefix
        *                  ~ Call _printf("words", ...) to get "[XX] words" (variadic, so you can use it like normal printf)
        *                  ~ Call pop_prefix() before you return from a method to restore the last one's prefix
        *                  ~ The prefix stack is managed, so you can never push or pop beyond the defined prefix stack size
        *                  ~ Saves over 10KB over manually putting "[XX] " at the beginning of every printed string

    The code was originally written for delta printers, but the probe calibration, surface normal, and grid-based Z correction are
    useful on other types as well. Some further improvements are needed, but I think I will eventually take "Delta" out of the
    name.

    G-codes:	G29	Probe Calibration
                G31	Heuristic Calibration (parallel simulated annealing)
                G32	Iterative Calibration (only calibrates endstops & delta radius)
                M667	Virtual shimming and depth correction params/enable/disable

    Files:	/sd/dm_surface_transform (contains depth map for use with depth map Z correction)

    The recommended way to use this on a Delta printer is:
    G29 (calibrate your probe)
    G32 (iterative calibration - gets endstops/delta radius correct - K to keep, but don't use that if you want to run G31 afterwards)
    G31 O P Q R S (simulated annealing - corrects for errors in X, Y, and Z - it may help to run this multiple times)
    G31 A (depth mapping - corrects errors in Z, but not X or Y - benefits from simulated annealing, though)

    The recommended way to use this on a Cartesian/CoreXY/SCARA etc. printer (of which the author has tested none, FYI):
    G29 (calibrate your probe)
    G31 A (depth mapping)

    To Do
    -------------------------
    * Migrate loads of arrays to AHB0
        * This is done, but none of the 2D arrays can be freed due to a possible bug in the memory pool manager.
        * See comments in AHB0_dealloc2Df(). It might be my fault, but I've put enough hours in on this already.
    * Audit arrays created during probe calibration & simulated annealing to see if any are candidates for AHB0 migration.
    * Increase probing grid size to 7x7, rather than 5x5. (Needs above AHB0 migration to be done first, crashes otherwise)
    * Elaborate probing grid to be able to support non-square grids (for the sake of people with rectangular build areas).
    * Add "leaning tower" support
      * Add {X, Y, Z(?)} coords for top and bottom of tower & use in FK and IK in robot/arm_solutions/LinearDeltaSolution.cpp/.h
      * Add letter codes to LinearDeltaSolution.cpp for saving/loading the values from config-override
      * Add simulated annealing section for tower lean
    * Make G31 B the specific command for heuristic calibration, and have it select O P Q R S (all annealing types) by default.
      * Make the annealer do a test probe rather than printing out the simulated depths.
      * If G31 B is run without args, use the last probed depths.
    * We are using both three-dimensional (Cartesian) and one-dimensional (depths type, .abs and .rel) arrays. Cartesians are
      necessary for IK/FK, but maybe we can make a type with X, Y, absolute Z, and relative Z, and be done with the multiple types.
      (Except for the depth map, which has to be kept in RAM all the time.)
      Such arrays can be "fat" while in use because they will live on the stack, and not take up any space when the calibration
      routines are not running.

*/


#include "ComprehensiveDeltaStrategy.h"
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
#include "SerialMessage.h"
#include "Vector3.h"
#include "Planner.h"
#include "utils.h"

#include <time.h>
#include <tuple>
#include <algorithm>
#include <string>

// probe_radius is "deprecated" in favor of just radius, but it shouldn't be.
// Using just "radius" sounds like the printer radius, but probing can't always be done that far out.
#define probe_radius_checksum CHECKSUM("probe_radius")

#define probe_smoothing_checksum      CHECKSUM("probe_smoothing")
#define probe_acceleration_checksum   CHECKSUM("probe_acceleration")
#define probe_priming_checksum        CHECKSUM("probe_priming")
#define probe_offset_x_checksum       CHECKSUM("probe_offset_x")
#define probe_offset_y_checksum       CHECKSUM("probe_offset_y")
#define probe_offset_z_checksum       CHECKSUM("probe_offset_z")

// Array subscripts: Cartesian axes
#define X 0
#define Y 1
#define Z 2

// Array subscripts: Towers and their counter-clockwise neighbors
#define XY 0
#define YZ 1
#define ZX 2



// This prints to ALL streams. If you have second_usb_serial_enable turned on, you better connect a terminal to it!
// Otherwise, eventually the serial buffers will get full and the printer may crash the effector into the build surface.

// Print "[PF] words", where PF is two characters sent to push_prefix()
#define _printf prefix_printf

// Print "words", no prefix
#define __printf THEKERNEL->streams->printf



// Init & clear memory on AHB0 for the bed-leveling depth map, and many other things.
// Thanks to ZGridStrategy.cpp, where I figured out how to use AHB0.
bool ComprehensiveDeltaStrategy::allocate_RAM(bool zero_pointers) {

    char error_str[] = "ERROR: Couldn't allocate RAM for";

    // If the printer is still booting, these objects will have random values in their memory addresses.
    // We need to zero them so that the following code blocks don't try to deallocate non-existent memory!
    if(zero_pointers) {

        best_probe_calibration = nullptr;
        surface_transform = nullptr;
        bili = nullptr;
        base_set = nullptr;
        cur_set = nullptr;
        temp_set = nullptr;
        cur_cartesian = nullptr;
        temp_cartesian = nullptr;
        test_point = nullptr;
        test_axis = nullptr;
        depth_map = nullptr;
        active_point = nullptr;

    } else {

        // Free up space if for some strange reason it's already allocated
        if(surface_transform != nullptr) {
            AHB0_dealloc_if_not_nullptr(surface_transform->depth);
        }
        
        AHB0_dealloc_if_not_nullptr(best_probe_calibration);
        AHB0_dealloc_if_not_nullptr(surface_transform);
        AHB0_dealloc_if_not_nullptr(bili);
        AHB0_dealloc_if_not_nullptr(base_set);
        AHB0_dealloc_if_not_nullptr(cur_set);
        AHB0_dealloc_if_not_nullptr(temp_set);
        AHB0_dealloc_if_not_nullptr(depth_map);
        AHB0_dealloc_if_not_nullptr(active_point);

        AHB0_dealloc2Df(test_point, DM_GRID_ELEMENTS);
        AHB0_dealloc2Df(test_axis, DM_GRID_ELEMENTS);
        AHB0_dealloc2Df(cur_cartesian, DM_GRID_ELEMENTS);
        AHB0_dealloc2Df(temp_cartesian, DM_GRID_ELEMENTS);

    }
    
    // Allocate space for surface_transform
    surface_transform = (struct surface_transform_t *)AHB0.alloc(sizeof(surface_transform_t));
    if(surface_transform == nullptr) {
        _printf("%s surface transform.\n", error_str);
        return false;
    }
    
    // Allocate space for depth map (this stores the most recent clean copy of the build surface depths)
    surface_transform->depth = (float *)AHB0.alloc(DM_GRID_ELEMENTS * sizeof(float));
    if(surface_transform->depth == nullptr) {
        _printf("%s depth map.\n", error_str);
        return false;
    }

    // Zero depth map & other variables
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        surface_transform->depth[i] = 0;
    }
    surface_transform->active = false;
    surface_transform->depth_enabled = false;
    surface_transform->have_depth_map = false;
    surface_transform->plane_enabled = false;
    surface_transform->have_normal = false;

    // Allocate space for lerp scratchpad
    bili = (struct bili_t *)AHB0.alloc(sizeof(bili_t)); // was sizeof(bili)
    if(bili == nullptr) {
        _printf("%s lerp scratchpad.\n", error_str);
        return false;
    }

    // Allocate space for best probe calibration settings
    best_probe_calibration = (struct best_probe_calibration_t*)AHB0.alloc(sizeof(best_probe_calibration_t));
    if(best_probe_calibration == nullptr) {
        _printf("%s best probe calibration.\n", error_str);
        return false;
    }

    // Allocate space for kinematics settings
    base_set = (KinematicSettings *)AHB0.alloc(sizeof(KinematicSettings));
    new(base_set) KinematicSettings;
    cur_set = (KinematicSettings *)AHB0.alloc(sizeof(KinematicSettings));
    new(cur_set) KinematicSettings;
    temp_set = (KinematicSettings *)AHB0.alloc(sizeof(KinematicSettings));
    new(temp_set) KinematicSettings;
    if(base_set == nullptr || cur_set == nullptr || temp_set == nullptr) {
        _printf("%s kinematic settings.\n", error_str);
        return false;
    }

    // Allocate space for annealing vars
    if((cur_cartesian = AHB0_alloc2Df(DM_GRID_ELEMENTS, 3)) == nullptr) {
        _printf("%s Cartesian grid.\n", error_str);
        return false;
    }

    if((temp_cartesian = AHB0_alloc2Df(DM_GRID_ELEMENTS, 3)) == nullptr) {
        _printf("%s Temp cartesian grid.\n", error_str);
        return false;
    }

    if((test_point = AHB0_alloc2Df(DM_GRID_ELEMENTS, 2)) == nullptr) {
        _printf("%s test points.\n", error_str);
        return false;
    }

    if((test_axis = AHB0_alloc2Df(DM_GRID_ELEMENTS, 3)) == nullptr) {
        _printf("%s test axes.\n", error_str);
        return false;
    }
    
    depth_map = (cds_depths_t *)AHB0.alloc(sizeof(cds_depths_t) * DM_GRID_ELEMENTS);
    if(depth_map == nullptr) {
        _printf("%s depth map.\n", error_str);
    }

    // Allocate space for active_point
    active_point = (test_point_enum_t *)AHB0.alloc(DM_GRID_ELEMENTS * sizeof(test_point_enum_t));
    if(active_point == nullptr) {
        _printf("%s active points.\n", error_str);
        return false;
    }

    // If we got this far, we should be all good!
    return true;

}


// Allocate a 2D array on AHB0
float** ComprehensiveDeltaStrategy::AHB0_alloc2Df(int rows, int columns) {

    // Allocate pointers to all the rows
    float **ptr = (float **)AHB0.alloc(rows * sizeof(float *));
    
    if(!ptr) {
        return nullptr;
    }

    // Allocate space for all the rows, and the columns they contain
    for(int i=0; i<rows; i++) {
        ptr[i] = (float*)AHB0.alloc(columns * sizeof(float));
        if(!ptr[i]) {
            return nullptr;
        }
    }
    return ptr;

}


// Deallocate 2D array from AHB0
bool ComprehensiveDeltaStrategy::AHB0_dealloc2Df(float** ptr, int rows) {

    /*

        This is a stub function, and it's going to stay a stub function for the forseeable future.
        There appears to be a bug in the deallocator. The last pointer it generates for me cannot
        be freed. I've double-checked this by printing out the row address pointers when they are
        allocated, and again when I try to free them. Every row pointer goes through AHB0.dealloc()
        just fine except for the last one, no matter how long the array is.
    
        I believe there may be a bug in the memory pool manager, like something that's being i++'d
        when it should be ++i'd, or similar. I don't feel like chasing that down, so whatever.
        
        I've rewritten all the code so that every 2D array is allocated at startup, and never freed.
        I apologize for wasting RAM on AHB0 like this, but at the moment I need to move on.

    */

    return false;

/*

_printf("dealloc: ptr=%p rows=%d\n", ptr, rows);
flush();

    if(ptr != nullptr) {
        for(int i=0; i<rows; i++) {
            if(ptr[i] != nullptr) {
_printf("Deallocating row at %p... ", ptr[i]);
flush();
                AHB0.dealloc(ptr[i]);
_printf("Done.\n");
flush();
            } else {
                __printf("AHB0_dealloc2Df: Row %d's pointer isn't null!\n", i);
                return false;
            }
        }
    }

_printf("Going to deallocate main pointer at %p.\n", ptr);
flush();
//    AHB0_dealloc_if_not_nullptr(ptr);

_printf("done.\n");
flush();



    return true;

*/

}


// Deallocate stuff from AHB0, but only if it's null
void ComprehensiveDeltaStrategy::AHB0_dealloc_if_not_nullptr(void *ptr) {
    if(ptr != nullptr) {
        AHB0.dealloc(ptr);
    }
}



// Destructor
ComprehensiveDeltaStrategy::~ComprehensiveDeltaStrategy() {

    if(surface_transform != nullptr) {
        AHB0_dealloc_if_not_nullptr(surface_transform->depth);
    }

    AHB0_dealloc_if_not_nullptr(surface_transform);
    AHB0_dealloc_if_not_nullptr(bili);
    AHB0_dealloc_if_not_nullptr(best_probe_calibration);
    AHB0_dealloc_if_not_nullptr(base_set);
    AHB0_dealloc_if_not_nullptr(cur_set);
    AHB0_dealloc_if_not_nullptr(temp_set);
    AHB0_dealloc_if_not_nullptr(test_point);
    AHB0_dealloc_if_not_nullptr(test_axis);
    AHB0_dealloc_if_not_nullptr(depth_map);
    AHB0_dealloc_if_not_nullptr(active_point);

    AHB0_dealloc2Df(test_point, DM_GRID_ELEMENTS);
    AHB0_dealloc2Df(test_axis, DM_GRID_ELEMENTS);
    AHB0_dealloc2Df(cur_cartesian, DM_GRID_ELEMENTS);
    AHB0_dealloc2Df(temp_cartesian, DM_GRID_ELEMENTS);

}



// printf() variant that can inject a prefix, and knows how to talk to the serial thingy
// Despite the extra space it takes, we still save a few KB from not having to store the same five characters ('[XX] ')
// at the beginning of a bunch of lines.
int __attribute__ ((noinline)) ComprehensiveDeltaStrategy::prefix_printf(const char* format, ...) {

    asm("");	// Discourage G++ from deciding to inline me anyway

    int len = strlen(format) * 2;
    char buf[len];

    va_list vl;
    va_start(vl, format);
    vsnprintf(buf, len, format, vl);
    va_end(vl);

    THEKERNEL->streams->printf("[%s] %s", method_prefix[method_prefix_idx], buf);
    
    return 1;

}





// This serves in place of a constructor; it will be called whenever the config is reloaded
// (which you can do over a serial console, by the way)
bool ComprehensiveDeltaStrategy::handleConfig() {

    // Allocate RAM for many things that we need to stuff into the AHB0 memory area
    if(!allocate_RAM(true)) {
        __printf("CDS: Unable to allocate RAM - giving up.\n");
        return false;
    }

    // Init method prefixes
    method_prefix_idx = -1;
    push_prefix("");

    // Set probe_from_height to a value that find_bed_center_height() will know means it needs to be initialized
    probe_from_height = -1;

    // Set the dirty flag, so we know we have to calibrate the endstops and delta radius
    geom_dirty = true;

    // Turn off Z compensation (we don't want that interfering with our readings)
    surface_transform->depth_enabled = false;
    surface_transform->have_depth_map = false;

    // Zero out the surface normal
    set_virtual_shimming(0, 0, 0);
    set_adjust_function(true);

    // Zero out depth_map
    zero_depth_maps();

    // Turn off all calibration types
    clear_calibration_types();

    // TODO: Read this from config_override via M-code
    surface_shape = PSS_CIRCLE;

    // Initialize the best probe calibration stats (we'll use sigma==-1 to check whether initialized)
    best_probe_calibration->sigma = -1;
    best_probe_calibration->range = -1;
    best_probe_calibration->accel = -1;
    best_probe_calibration->debounce_count = -1;
    best_probe_calibration->decelerate = false;
    best_probe_calibration->eccentricity = true ;
    best_probe_calibration->smoothing = -1;
    best_probe_calibration->fast = -1;
    best_probe_calibration->slow = -1;

    // Probe radius
    float r = THEKERNEL->config->value(leveling_strategy_checksum, comprehensive_delta_strategy_checksum, probe_radius_checksum)->by_default(-1)->as_number();
    if(r == -1) {
        // Deprecated config syntax
        r =  THEKERNEL->config->value(zprobe_checksum, probe_radius_checksum)->by_default(100.0F)->as_number();
    }
    this->probe_radius = r;

    // Initialize bilinear interpolation array scaler (requires probe_radius)
    bili->cartesian_to_array_scaler = (DM_GRID_DIMENSION - 1) / (probe_radius * 2);

    // Initialize test points (requires probe_radius)
    init_test_points();

    // Probe smoothing: If your probe is super jittery, we can probe multiple times per request and average the results
    int p = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_smoothing_checksum)->by_default(1)->as_number();
    if(p <  1) p = 1;
    if(p > 10) p = 10;
    this->probe_smoothing = p;

    // Probe priming: Run the probe a specified # of times before the "real" probing (good for printers that demonstrate a Z settling issue)
    p = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_priming_checksum)->by_default(0)->as_number();
    if(p <  0) p = 0;
    if(p > 10) p = 10;
    this->probe_priming = p;

    // Probe acceleration
    probe_acceleration = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_acceleration_checksum)->by_default(200)->as_number();

    // Effector coordinates when probe is at bed center, at the exact height where it triggers.
    // To determine this:
    // - Heat the extruder
    // - Jog it down to the print surface, so it leaves a little dot
    // - Deploy the probe and move it until its trigger is touching the dot
    // - Jog the probe up enough to remove the dot, and then do so
    // - Jog the probe back down again until it triggers (use tiny moves to get it as accurate as possible)
    // - Record the position in config as probe_offset_x/y/z
    this->probe_offset_x = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_x_checksum)->by_default(0)->as_number();
    this->probe_offset_y = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_y_checksum)->by_default(0)->as_number();
    this->probe_offset_z = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_z_checksum)->by_default(0)->as_number();

    return true;

}


// Process incoming G- and M-codes
bool ComprehensiveDeltaStrategy::handleGcode(Gcode *gcode) {

    if(gcode->has_g) {
        // G code processing
        if(gcode->g == 29) { // Test the Z-probe for repeatability
            THEKERNEL->conveyor->wait_for_empty_queue();
            measure_probe_repeatability(gcode);
            return true;
        }

        if(gcode->g == 31) { // Depth mapping & heuristic delta calibration


        /*
        // Code for verifying the test points
        _printf("There are %d grid elements.\n \n", DM_GRID_ELEMENTS);
        for(int i=0; i<DM_GRID_ELEMENTS; i++) {

            _printf("Test point[%d]: X=%1f, Y=%1f\n", i, test_point[i][X], test_point[i][Y]);
            flush();
            
        }

        return false;
        */


        /*
        // As of now, this works.

        __printf("Beginning 2D array test\n");
        flush();

        int i,j;
        int rows=3, cols=3;
        float **array;
        array = (float**)AHB0.alloc(rows * sizeof(float *));
        for (i = 0; i < rows; i++) {
          array[i] = (float*)AHB0.alloc(cols * sizeof(float));
        }

        // Some testing
        for (i = 0; i < rows; i++) {
          for (j = 0; j < cols; j++)
            array[i][j] = (i * j) + 0.545; // or whatever you want
        }

        for (i = 0; i < rows; i++) {
          for (j = 0; j < cols; j++) {
            __printf("row=%d col=%d val=%3.1f\n", i, j, array[i][j]);
            flush();
          }
          newline();

        }

        __printf("Test complete\n");
        flush();

        return true;
        /**/



        /*

        // As of now, this also works!

        _printf("AHB0.alloc() test\n");

        flush();
        float** test;
        test = AHB0_alloc2Df(41, 3);
        _printf("Pointer created\n");
        flush();

        _printf("Setting values...\n");
        flush();
        for(int row=0; row<41; row++) {
            for(int col=0; col<3; col++) {
                test[row][col] = row + col;
                __printf("%1f ", test[row][col]);
                flush();
            }
            __printf("\n");
            flush();
        }

        _printf("Deallocating...\n");
        flush();
        AHB0_dealloc2Df(test, 41);
        _printf("Done.\n");


        return true;
        */


            return handle_depth_mapping_calibration(gcode);
        }

        if(gcode->g == 32) { // Auto calibration for delta, Z bed mapping for cartesian
            bool keep = false;
            if(gcode->has_letter('K')) {
                keep = gcode->get_value('K');
            }
            THEKERNEL->conveyor->wait_for_empty_queue();
            iterative_calibration(keep);
            return true;
        }

    } else if(gcode->has_m) {
    
        char letters[] = "ABCDEFTUVLR";

        switch(gcode->m) {

            // If the geometry is modified externally, we set the dirty flag (but not for Z - that requires no recalibration)
            case 665:
                for(unsigned int i=0; i<strlen(letters); i++) {
                    if(gcode->has_letter(letters[i])) {
                        geom_dirty = true;
                    }
                }
                break;
            
            // Set geom dirty on trim change as well
            case 666:
                geom_dirty = true;
                break;
            
            // Surface equation for virtual shimming, depth map correction, and master enable
            case 667:
                handle_shimming_and_depth_correction(gcode);
                break;

            // Save depth map (CSV)
            case 500:
            case 503:
                // We use gcode->stream->printf instead of _printf because the dispatcher temporarily replaces the serial
                // stream printer with a file stream printer when M500/503 is sent.
                // A=X, B=Y, C=Z, D=Shimming enabled (1 or 0), E=Depth map correction enabled (1 or 0), Z=Master enable (1 or 0)
                // Master enable has to be on for either shimming or depth map correction to actually work.
                // Their individual flags only control whether they're available or not.
                gcode->stream->printf(
                    ";ABC=Shimming data; D=Shimming; E=Depth map; Z=Master enable\nM667 A%1.4f B%1.4f C%1.4f D%d E%d Z%d\n",
                    surface_transform->tri_points[X][Z], surface_transform->tri_points[Y][Z], surface_transform->tri_points[Z][Z],
                    (int)surface_transform->plane_enabled, (int)surface_transform->depth_enabled, (int)surface_transform->active);
                break;

        } // switch(gcode->m)
        
    }

    return false;

}


// Handlers for G-code commands too elaborate (read: stack-heavy) to cleanly fit in handleGcode()
// This fixes config-override file corruption when doing M500. :)

// G31
bool ComprehensiveDeltaStrategy::handle_depth_mapping_calibration(Gcode *gcode) {

    THEKERNEL->conveyor->wait_for_empty_queue();

    if(gcode->has_letter('F')) {
        __printf("[!!] Forcing re-probe.\n");
        geom_dirty = true;
    }

    if(gcode->has_letter('A')) {

        push_prefix("DC");
        print_task_with_warning("Depth-mapping calibration");

        if(handle_z_correction()) {

            _printf("Checking calibration...\n");
            if(!depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED, false)) {
                _printf("Couldn't depth-map the surface.\n");
                geom_dirty = true;
                return false;
            }

            _printf("/!\\ IMPORTANT /!\\ Type M500 to save!\n");

        } else {

            geom_dirty = true;
            return false;

        }

        geom_dirty = true;

        zprobe->home();
        pop_prefix();


    } else if(gcode->has_letter('Z')) {

        // We are only here to map the surface - no calibration
        newline();
        push_prefix("DM");
        print_kinematics();

        if(!depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED, false)) {
            _printf("Couldn't depth-map the surface.\n");
        }
        pop_prefix();
        zprobe->home();

    } else {

        // Do a heuristic calibration (or simulation)
        clear_calibration_types();
        int annealing_tries = 50;
        float max_temp = 0.35;
        float binsearch_width = 0.1;
        float overrun_divisor = 2;
        bool simulate_only = false;
        bool keep_settings = false;
        bool zero_all_offsets = false;
        bool set_geom_after_each_caltype = true;

        // Keep settings?
        if(gcode->has_letter('K')) {
            keep_settings = true;
        }

        // Simulate-only
        if(gcode->has_letter('L')) {
            simulate_only = true;
        }

        // Endstops
        if(gcode->has_letter('O')) {
            caltype.endstop.active = true;
            caltype.endstop.annealing_temp_mul = gcode->get_value('O');
        }
        
        // Delta radius, including individual tower offsets
        if(gcode->has_letter('P')) {
            caltype.delta_radius.active = true;
            caltype.delta_radius.annealing_temp_mul = gcode->get_value('P');
        }

        // Arm length, including individual arm length offsets
        if(gcode->has_letter('Q')) {
            caltype.arm_length.active = true;
            caltype.arm_length.annealing_temp_mul = gcode->get_value('Q');
        }

        // Tower angle offsets
        if(gcode->has_letter('R')) {
            caltype.tower_angle.active = true;
            caltype.tower_angle.annealing_temp_mul = gcode->get_value('R');
        }
        
        // Surface plane virtual shimming
        if(gcode->has_letter('S')) {
            caltype.virtual_shimming.active = true;
            caltype.virtual_shimming.annealing_temp_mul = gcode->get_value('S');
        }

        // Annealing tries
        // Generally, more iterations require lower temps
        if(gcode->has_letter('T')) {
            annealing_tries = gcode->get_value('T');
        }
        
        // Max temperature (tradeoff between "too cold to get there" and "so hot that it boils" - you want "just right")
        if(gcode->has_letter('U')) {
            max_temp = gcode->get_value('U');
        }
        
        // Binary search width (tradeoff between speed and accuracy - I recommend 0.1)
        if(gcode->has_letter('V')) {
            binsearch_width = gcode->get_value('V');
        }

        // Overrun divisor (what a random move is divided by if it overshoots the ideal value)
        // No, it isn't a good idea to use <=1.
        if(gcode->has_letter('W')) {
            overrun_divisor = gcode->get_value('W');
        }

        // Zero all offset values
        if(gcode->has_letter('Y')) {
            zero_all_offsets = true;
        }

        // Set geometry after each caltype
        if(gcode->has_letter('J')) {
            set_geom_after_each_caltype = false;
        }

        push_prefix("HC");
        if(gcode->get_num_args() > 0) {

            // Make sure at least one caltype is turned on
            if(
                !caltype.endstop.active &&
                !caltype.delta_radius.active &&
                !caltype.arm_length.active &&
                !caltype.tower_angle.active &&
                !caltype.virtual_shimming.active
            ){
                _printf("No calibration types selected - activating endstops & delta radius.\n");
                caltype.endstop.active = true;
                caltype.delta_radius.active = true;
            }

            // OK! Run the simulated annealing algorithm.
            heuristic_calibration(annealing_tries, max_temp, binsearch_width, simulate_only, keep_settings, zero_all_offsets, overrun_divisor, set_geom_after_each_caltype);
        
        } else {

            // No args given, so show instructions
            print_g31_help();

        } //if(gcode->get_num_args() > 0)
        pop_prefix();
    
    } // !gcode->has_letter('M')

    return true;

}


// Do the depth map-based calibration (fixes Z only, not X or Y)
bool ComprehensiveDeltaStrategy::handle_z_correction() {

    int x, y;

    // It took me a really, really long (and frustrating) time to figure this out
    if(probe_offset_x != 0 || probe_offset_y != 0) {
        _printf("Depth correction doesn't work with X or Y probe offsets.\n");
        return false;
    }

    _printf("Probing bed for depth correction...\n");

    // Disable depth correction (obviously)
    surface_transform->depth_enabled = false;

    // Build depth map
    if(!depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED, true)) {
        _printf("Couldn't build depth map - aborting!\n");
        pop_prefix();
        return false;
    }

    // Copy depth map to surface_transform->depth[], which contains depths only
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        surface_transform->depth[i] = cur_cartesian[i][Z];
    }

    // Propagate values outward from circle to edge, in case they go outside probe_radius
    if(surface_shape == PSS_CIRCLE) {
        for(y=0; y<DM_GRID_DIMENSION; y++) {
            for(x=0; x <= (DM_GRID_DIMENSION-1) / 2; x++) {

                int dm_pos_right = (y * DM_GRID_DIMENSION) + ((DM_GRID_DIMENSION - 1) / 2) + x;
                int dm_pos_left  = (y * DM_GRID_DIMENSION) + ((DM_GRID_DIMENSION - 1) / 2) - x;

                // Propagate right
                if(active_point[dm_pos_right] == TP_INACTIVE) {
                    surface_transform->depth[dm_pos_right] = surface_transform->depth[dm_pos_right - 1];
                }

                // Propagate left
                if(active_point[dm_pos_left] == TP_INACTIVE) {
                    surface_transform->depth[dm_pos_left] = surface_transform->depth[dm_pos_left + 1];
                }

            }
        }
    }

    // Enable depth correction
    surface_transform->depth_enabled = true;
    set_adjust_function(true);

    // Save to a file.
    // I tried saving this with G-codes, but I guess you can't stuff that much data.
    // The config-overrides file was corrupted when I tried! I found mention of a
    // file corruption bug elsewhere in the firmware, so I guess it's a known issue.
    // I could have just written everything as binary data, but I wanted people to
    // be able to populate the file with numbers from a regular $10 depth gauge in
    // case they don't have a Z-probe.
    FILE *fp = fopen("/sd/dm_surface_transform", "w");
    if(fp != NULL) {

        fprintf(fp, "; Depth Map Surface Transform\n");
        //__printf("; Depth Map Surface Transform\n");
        flush();
        for(y=0; y<DM_GRID_DIMENSION; y++) {
            fprintf(fp, "; Line %d of %d\n", y + 1, DM_GRID_DIMENSION);
            //__printf("; Line %d of %d\n", y + 1, DM_GRID_DIMENSION);
            flush();
            for(x=0; x<DM_GRID_DIMENSION; x++) {
                fprintf(fp, "%1.5f\n", surface_transform->depth[(y * DM_GRID_DIMENSION) + x]);
                //__printf("%1.5f\n", surface_transform->depth[(y * DM_GRID_DIMENSION) + x]);
                flush();
            }
        }

        // This is probably important to do
        fclose(fp);
        flush();

        _printf("Surface transform saved to SD card.\n");
    
    } else {

        _printf("Couldn't save surface transform to SD card!\n");

    }

    // Dirty the geom again in case they decide to run G31 OPQRS[...] again
    geom_dirty = true;

    return true;

}


// This probably eats a ton of stack, so it gets its own method
void __attribute__ ((noinline)) ComprehensiveDeltaStrategy::print_g31_help() {

    asm("");	// A subtle hint to the compiler: DON'T inline this

    flush();
    _printf("G31 usage: (* = you can supply an annealing multiplier)\n");
    _printf("Z: Probe and display depth map - no calibration\n");
    _printf("A: Set up depth map for auto leveling (corrects Z only - run AFTER annealing)\n");
    _printf("\n");
    flush();
    _printf("Simulated annealing (corrects X, Y and Z - run G32 first):\n");
    _printf("J: Update geometry after ALL vars are annealed each pass, rather than after each one is annealed\n");
    _printf("K: Keep last settings\n");
    _printf("L: Simulate only (don't probe)\n");
    _printf("O: Endstops *\n");
    flush();
    _printf("P: Delta radius *\n");
    _printf("Q: Arm length *\n");
    _printf("R: Tower angle offsets *\n");
    _printf("S: Surface plane virtual shimming *\n");
    flush();
    _printf("t: Annealing: Iterations (50)\n");		// Repetier Host eats lines starting with T >:(
    _printf("U: Annealing: Max t_emp (0.35)\n");	// Repetier Host eats all lines containing "temp" >8(
    _printf("V: Annealing: Binary search width (0.1)\n");
    _printf("W: Annealing: Overrun divisor (2)\n");
    _printf("Y: Zero all individual radius, angle, and arm length offsets\n");
    flush();

}

// M667
bool ComprehensiveDeltaStrategy::handle_shimming_and_depth_correction(Gcode *gcode) {

    push_prefix("DM");

    // Triangle points for shimming surface normal
    if(gcode->has_letter('A')) surface_transform->tri_points[X][Z] = gcode->get_value('A');
    if(gcode->has_letter('B')) surface_transform->tri_points[Y][Z] = gcode->get_value('B');
    if(gcode->has_letter('C')) surface_transform->tri_points[Z][Z] = gcode->get_value('C');

    // Shimming
    if(gcode->has_letter('D')) surface_transform->plane_enabled = gcode->get_value('D');
    if(surface_transform->plane_enabled) {
        set_virtual_shimming(surface_transform->tri_points[X][Z], surface_transform->tri_points[Y][Z], surface_transform->tri_points[Z][Z]);
        set_adjust_function(true);
    }

    // Depth map
    if(gcode->has_letter('E')) {

        if(probe_offset_x == 0 && probe_offset_y == 0) {

            if(surface_transform->have_depth_map) {

                // Depth map already loaded 
                surface_transform->depth_enabled = gcode->get_value('E');

            } else {

                // ST not initialized - try to load it
                FILE *fp = fopen("/sd/dm_surface_transform", "r");
                if(fp != NULL) {

                    char buf[48];
                    int i = 0;
                    
                    while(fgets(buf, sizeof buf, fp) != NULL) {

                        // Chop trailing newline
                        char *pos;
                        if((pos=strchr(buf, '\n')) != NULL) {
                            *pos = '\0';
                        }

                        // Skip comment lines
                        if(buf[0] == ';') continue;

                        // Add float value to the transform                            
                        if(i < DM_GRID_ELEMENTS) {

                            float fval = atof(buf);

                            if(fval > -5 && fval < 5) {
                                surface_transform->depth[i] = strtof(buf, NULL);
                                //_printf("buffer='%s' - Surface transform element %2d set to %1.3f.\n", buf, i, surface_transform->depth[i]);
                                i++;
                            } else {
                                _printf("Surface transform element %2d is out of range (%1.3f) - aborting.\n", i, surface_transform->depth[i]);
                                fclose(fp);
                                surface_transform->depth_enabled = false;
                                return false;
                            }

                        }
                    } // while

                    // Goodbye, cool file full of useful numbers
                    fclose(fp);

                    // Sanity check
                    if(i != DM_GRID_ELEMENTS) {
                        _printf("ERROR: Expected %d elements, but got %d - aborting.\n", DM_GRID_ELEMENTS, i);
                        surface_transform->have_depth_map = false;
                        surface_transform->depth_enabled = false;
                    } else {
                        surface_transform->depth_enabled = gcode->get_value('E');
                        if(surface_transform->depth_enabled == true) {
                            surface_transform->depth_enabled = true;
                            set_adjust_function(true);
                        } else {
                            surface_transform->depth_enabled = false;
                        }
                    }

                } else {

                    _printf("Depth correction not initialized.\n");

                } // if(fp != NULL)

            } // if(surface_transform->have_depth_map)

        } else {

            // FIXME:
            // For now, silently fail to enable.
            // This is because whatever we spew here risks hanging the firmware on startup,
            // because it will fill a serial buffer that never gets flushed.
            // The same warning is printed above if you do G31 A with probe offsets enabled,
            // so users are somewhat likely to see it.
            
            //_printf("Depth correction doesn't work with X or Y probe offsets.\n");

        } // if(probe offsets are 0)
      
    } // if(gcode->has_letter('E')

    // Global enable/disable
    if(gcode->has_letter('Z')) {
        bool enable = gcode->get_value('Z');
        if(enable) {
            if(surface_transform->depth_enabled || surface_transform->plane_enabled) {
                set_adjust_function(true);
            } else {
                _printf("Can't enable surface transform - no data.\n");
            }
        } else {
            set_adjust_function(false);
        }
    }

    //_printf("Surface transform: Depth map=%s; Surface plane=%s; Active=%s\n", surface_transform->depth_enabled ? _STR_ENABLED_ : _STR_DISABLED_, surface_transform->plane_enabled ? _STR_ENABLED_ : _STR_DISABLED_, surface_transform->active ? _STR_ENABLED_ : _STR_DISABLED_);
    pop_prefix();

    return true;

}


// Main heuristic calibration routine
// This expects caltype.*.active to be set true/false beforehand
bool ComprehensiveDeltaStrategy::heuristic_calibration(int annealing_tries, float max_temp, float binsearch_width, bool simulate_only, bool keep_settings, bool zero_all_offsets, float overrun_divisor, bool set_geom_after_each_caltype) {


    /*

        Simulated Annealing Notes
        
        - Works by trying to take the system from a high-energy state to the lowest-energy state
        - Slowly reduces the "temperature" of the system
            - Temperature affects how "bad" a possibility can be and still be tested
        - Acceptance probability function
            - P(e, e', T)
                -  e: existing state
                - e': candidate test state
                -  T: global temperature
            - Generally, but not always, we want e' < e
                - If e' > e, it's "hotter" and less desirable
                - However, hotter may be necessary to escape a local optimum
                - How much hotter e' can be than e is bound by T
            - P(e, e', T) must ALWAYS be positive
                - If it's not, we may get stuck around a local optimum and never "escape" to find the global optimum
        
        - Pseudocode
            - state = state[0]					// OK
            - energy = energy(state)				// OK
            - kMax = max iterations				// OK
            - eMax = maximum acceptable energy			// OK
            - while(k < kMax && energy > eMax) {		// OK
            -	temp = temperature(k / kMax)			// OK
            -	stateNew = randomNeighbor(s)			// Pick some random other state w/ variables anywhere in range
            - 	energyNew = energy(stateNew)			// OK
            -	if(P(energy, energyNew, temp) > frand(0, 1)) {	// Simulate energy of new state, compare to temperature
            -		state = stateNew;			// OK
            -		energy = energyNew;			// OK
            - 	}						// OK
            -	k++						// OK
            - }							// OK

    */


    // We'll be needing some random numbers
    srand(clock());

    // Banner
    push_prefix("HC");
    print_task_with_warning("Heuristic calibration");

    // Sanity check regular variables
    annealing_tries = clamp(annealing_tries, 10, 1000);
    max_temp = clamp(max_temp, 0, 2);
    binsearch_width = clamp(binsearch_width, 0, 0.5);
    overrun_divisor = clamp(overrun_divisor, 0.5, 15);

    // Ensure parallel annealing temp multipliers aren't zero
    if(caltype.endstop.annealing_temp_mul == 0) caltype.endstop.annealing_temp_mul = 1;
    if(caltype.delta_radius.annealing_temp_mul == 0) caltype.delta_radius.annealing_temp_mul = 1;
    if(caltype.arm_length.annealing_temp_mul == 0) caltype.arm_length.annealing_temp_mul = 1;
    if(caltype.tower_angle.annealing_temp_mul == 0) caltype.tower_angle.annealing_temp_mul = 1;
    if(caltype.virtual_shimming.annealing_temp_mul == 0) caltype.virtual_shimming.annealing_temp_mul = 1;
    
    // Ensure parallel annealing temp multipliers aren't crazy
    caltype.endstop.annealing_temp_mul = clamp(caltype.endstop.annealing_temp_mul, 0, 50);
    caltype.delta_radius.annealing_temp_mul = clamp(caltype.delta_radius.annealing_temp_mul, 0, 50);
    caltype.arm_length.annealing_temp_mul = clamp(caltype.arm_length.annealing_temp_mul, 0, 50);
    caltype.tower_angle.annealing_temp_mul = clamp(caltype.tower_angle.annealing_temp_mul, 0, 50);
    caltype.virtual_shimming.annealing_temp_mul = clamp(caltype.virtual_shimming.annealing_temp_mul, 0, 50);

    // Zero offsets, if requested
    if(zero_all_offsets) {
        set_virtual_shimming(0, 0, 0);
        set_trim(0, 0, 0);
        set_tower_radius_offsets(0, 0, 0);
        set_tower_angle_offsets(0, 0, 0);
        get_kinematics(base_set);
        get_kinematics(cur_set);
    }

    // Is it live, or is it Memorex?
    char _sim[] = "Simulation (L)";
    char _probe[] = "Probe";
    _printf("            Data source: %s\n", simulate_only ? _sim : _probe);

    // Display values used, along with the G-codes used to set them
    _printf("           Active tests: ");
    display_calibration_types(true, false);
    _printf("         Inactive tests: ");
    display_calibration_types(false, true);

    _printf("Set geom during/after (J): %s\n", set_geom_after_each_caltype ? "During" : "After");
    _printf("   Keep last settings (K): %s\n", keep_settings ? _STR_TRUE_ : _STR_FALSE_);
    _printf("      Annealing tries (T): %d\n", annealing_tries);
    _printf("             Max temp (U): %1.3f\n", max_temp);
    _printf("  Binary search width (V): %1.3f\n", binsearch_width);
    _printf("      Overrun divisor (W): %1.3f\n", overrun_divisor);
    _printf("     Zero all offsets (Y): %s\n", zero_all_offsets ? _STR_TRUE_ : _STR_FALSE_);
    newline();

    // Make sure the depth maps are blank
    //zero_depth_maps();


    // *******************************************************************
    // * Run a simulated annealing to get the printer config most likely *
    // * to produce what the real printer is doing                       *
    // *******************************************************************

    // Depth correction has to be off, or none of this stuff will work
    surface_transform->depth_enabled = false;
    
    // Deal with virtual shimming
    if(caltype.virtual_shimming.active) {
        surface_transform->plane_enabled = true;
    } else {
        surface_transform->plane_enabled = false;
    }

    // We need to save the kinematic settings for later
    if(!simulate_only || !base_set->initialized) {
        _printf("Baseline kinematics updated.\n");
        get_kinematics(base_set);
    }

    // Make sure cur_set is initialized
    if(!cur_set->initialized) {
        get_kinematics(cur_set);
    }

    // If we aren't keeping the kinematic settings, copy the base settings into the current settings
    // If not simulating, we need to stay with the last kinematics because they may have changed
    // (whereas, in simulation, they never change)
    if(keep_settings || !simulate_only) {
        _printf("Keeping existing kinematics.\n");
        get_kinematics(cur_set);
    } else {
        _printf("Restoring baseline kinematics.\n");
        base_set->copy_to(cur_set);
        set_kinematics(cur_set);
    }


    // Tests (min, max, value|TEST_INIT_MIDRANGE))
    // Main tests:
    TestConfig test_endstop[3] { {-5, 0}, {-5, 0}, {-5, 0} };
    TestConfig test_delta_radius(cur_set->delta_radius - 5, cur_set->delta_radius + 5);
    TestConfig test_arm_length(cur_set->arm_length - 5, cur_set->arm_length + 5);
    TestConfig test_tower_angle[3] { {-3, 3}, {-3, 3}, {-3, 3} };
    TestConfig test_virtual_shimming[3] { {-3, 3}, {-3, 3}, {-3, 3} };

    // Offsets that tie into the main tests:
    TestConfig test_delta_radius_offset[3] { {-3, 3}, {-3, 3}, {-3, 3} };
//    TestConfig test_arm_length_offset[3] { {-3, 3}, {-3, 3}, {-3, 3} };

    // Set up for outer loop
    int outer_try, outer_tries = 1;		// How many full iterations (probe print surface and run annealing for each test variable)
    int annealing_try;				// Current annealing iteration
    float tempFraction, temp;
    float best_value;				// Binary search returns this

    // Set up target tolerance
    float target = 0.005;			// Target deviation for individual element in simulated annealing only
    float global_target = 0.010;		// Target Z-deviation for all points on print surface

    // If simulating, we only need to run the numbers once per session. Otherwise, they have to be redone every time.
    static bool need_to_simulate_IK = true;

    // Copy kinematic settings from temp_set to cur_set after simulating IK?
    bool restore_from_temp_set;

    // Other vars
//    float cur_cartesian[DM_GRID_ELEMENTS][3];
    int j, k;
    int try_mod_5;				// Will be set to annealing_try % 5
    float lowest;				// For finding the lowest absolute value of three variables

    // Keep track of energy so that we can bail if the annealing stalls
    #define LAST_ENERGY_N 6
    float last_energy[LAST_ENERGY_N];
    unsigned last_energy_count = 0;
    for(j=0; j<LAST_ENERGY_N; j++) {
        last_energy[j] = 0;
    }

    
    // ************************************
    // * Simulated Annealing - Outer Loop *
    // ************************************

    for(outer_try = 0; outer_try < outer_tries; outer_try++) {

        // Clear flag that tells the IK simulator to restore kinematics to temp_set after the simulation runs
        restore_from_temp_set = false;


        if(simulate_only) {

            // Doing it for pretend: Generate some test values
            zero_depth_maps();

            if(!keep_settings) {

                _printf("Perturbing simulated printer parameters.\n");

                // Save existing kinematics
                restore_from_temp_set = true;
                get_kinematics(temp_set);

                // Perturb the parameters
                if(caltype.endstop.active) {
                    set_trim(-1.834, -1.779, 0.000);
                }

                if(caltype.delta_radius.active) {
                    set_delta_radius(131.25);
                    set_tower_radius_offsets(-1, 0, 2);
                } else {
                    set_tower_radius_offsets(0, 0, 0);
                }

                if(caltype.arm_length.active) {
                    set_arm_length(269.75);
                }

                if(caltype.tower_angle.active) {
                    set_tower_angle_offsets(1, 0, -1.5);
                } else {
                    set_tower_angle_offsets(0, 0, 0);
                }
                
                if(caltype.virtual_shimming.active) {
                    set_virtual_shimming(0.0, 0.0, -1.0);
                } else {
                    set_virtual_shimming(0, 0, 0);
                }

                // Save the perturbed kinematics
                get_kinematics(cur_set);

                // Trigger regen of carriage positions
                need_to_simulate_IK = true;

                _printf("Done hosing the variables.\n");
                print_kinematics();

            } // !keep_settings

        } else { // !simulate_only

            // Doing it for real: Get values from probe
            // depth_map[] will contain measured depths relative to center

            if(!keep_settings) {

                _printf("Depth-mapping the print surface...\n");
                print_kinematics();
                if(!depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED, false)) {
                    _printf("Couldn't depth-map the surface.\n");
                    zprobe->home();
                    pop_prefix();
                    return false;
                }

            } else {

                _printf("/!\\ Keeping existing depth map.\n");

            }

        } // simulate_only


        // ***************************************************************
        // * Figure out the actuator positions,                          *
        // * given a printer that ~perfectly~ matches the current config *
        // ***************************************************************

        // Generated test positions => cur_cartesian, generated axis positions => test_axis[] (class member)
        if(need_to_simulate_IK || !simulate_only) {
            _printf("Generating carriage positions for a printer with this configuration.\n");

            simulate_IK(cur_cartesian, cur_set->trim);
            if(restore_from_temp_set) {
                temp_set->copy_to(cur_set);
                set_kinematics(cur_set);
            }
            need_to_simulate_IK = false;
        }

        newline();
        _printf("Starting test configuration: Arm Length=%1.3f, Delta Radius=%1.3f\n", cur_set->arm_length, cur_set->delta_radius);

        // Get energy of initial state
        float energy = calc_energy(cur_cartesian);
        newline();
        _printf("***** Simulated annealing pass %d of %d in progress *****\n", outer_try + 1, outer_tries);
        _printf("Existing calibration has energy %1.3f\n \n", energy);
        _printf("Reticulating splines...\n");

        // For deciding the random order in which to run the calibrations (it's different each time
        // in order to eliminate a potential source of bias)
        unsigned char caltype_order[CDS_N_CALTYPES];
        for(int c=0; c<CDS_N_CALTYPES; c++) {
            caltype_order[c] = c;
        }
        

        // ************************************
        // * Simulated Annealing - Inner Loop *
        // ************************************

        for(annealing_try=0; annealing_try<annealing_tries; annealing_try++) {

            // Shuffle caltype list
            // Disabled - this really doesn't help, at all. In fact, I get slightly worse results with it!!!
            // Would be interesting to see whether a weak AI could figure out the optimal order on a per-machine basis.
/*
            int iter = 3;
            for(int i=0; i<CDS_N_CALTYPES * iter; i++) {
                float rnd = (float)rand() / RAND_MAX;
                int j = rnd * CDS_N_CALTYPES;
                int tmp = caltype_order[i % iter];
                caltype_order[i % iter] = caltype_order[j];
                caltype_order[j] = tmp;
            }
*/

            // Twiddle an LED so the user knows we aren't dead.
            // From main.c: "led0 init doe, led1 mainloop running, led2 idle loop running, led3 sdcard ok"
            // Therefore, LED 1 seems like the one to strobe. Normally, it's constantly dark when this method is running.
            // We will blink LED 2 as each calibration type (endstops, delta radius, etc.) is annealed.
            blink_LED(1);

            // Set the annealing temperature
            tempFraction = (float)annealing_try / (float)annealing_tries;
            temp = max_temp - (tempFraction * max_temp);
            if(temp < 0.01) {
                temp = 0.01;
            }
            
            try_mod_5 = annealing_try % 5;
            float local_temp;

            for(int cal_type=0; cal_type<CDS_N_CALTYPES; cal_type++) {

                switch(caltype_order[cal_type]) {

                    case CT_ENDSTOP:
//                        _printf("~ Endstop ~\n"); flush();
                        
                        // ************
                        // * Endstops *
                        // ************

                        if(caltype.endstop.active) {

                            local_temp = temp * caltype.endstop.annealing_temp_mul;

                            for(k=0; k<3; k++) {
                                best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_test_trim, cur_set->trim, k, local_temp, test_endstop[k].range_min, test_endstop[k].range_max, binsearch_width, cur_cartesian, target);
                                move_randomly_towards(cur_set->trim[k], best_value, local_temp, target, overrun_divisor);

                                // Set trim
                                if(set_geom_after_each_caltype) {
                                    set_trim(cur_set->trim[X], cur_set->trim[Y], cur_set->trim[Z]);
                                }

                            } // k

                        } // caltype.endstop

                        blink_LED(2);
                        break;

                    case CT_DELTA_RADIUS:
//                        _printf("~ DR ~\n"); flush();
                        // ****************
                        // * Delta Radius *
                        // ****************

                        if(caltype.delta_radius.active) {

                            local_temp = temp * caltype.delta_radius.annealing_temp_mul;

                            // Find the best tower (delta) radius offsets
                            for(k=0; k<3; k++) {
                                best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_tower_radius_offsets, cur_set->tower_radius, k, local_temp, test_delta_radius_offset[k].range_min, test_delta_radius_offset[k].range_max, binsearch_width, cur_cartesian, target);
                                move_randomly_towards(cur_set->tower_radius[k], best_value, local_temp, target, overrun_divisor);

                                // Find the tower radius with the lowest absolute value
                                lowest = 999;
                                for(k=0; k<3; k++) {
                                    if(fabs(cur_set->tower_radius[k]) < lowest) {
                                        lowest = cur_set->tower_radius[k];
                                    }
                                }

                                // Steal that value from the individual radius settings and give it to the global radius setting
                                for(k=0; k<3; k++) {
                                    cur_set->tower_radius[k] -= lowest;
                                }
                                cur_set->delta_radius += lowest;


                                // Update the robot
                                if(set_geom_after_each_caltype) {
                                    set_delta_radius(cur_set->delta_radius, false);
                                    set_tower_radius_offsets(cur_set->tower_radius[X], cur_set->tower_radius[Y], cur_set->tower_radius[Z], false);
                                }

                            } // k

                        } // caltype.delta_radius

                        blink_LED(2);
                        break;

                    case CT_ARM_LENGTH:
//                        _printf("~ Arm Length ~\n"); flush();
                        // **************
                        // * Arm Length *
                        // **************

                        if(caltype.arm_length.active) {

                            local_temp = temp * caltype.arm_length.annealing_temp_mul;
                            best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_arm_length, cur_set->arm_length, local_temp, test_arm_length.range_min, test_arm_length.range_max, binsearch_width, cur_cartesian, target);
                            move_randomly_towards(cur_set->arm_length, best_value, local_temp, target, overrun_divisor);

                            // Update the robot
                            if(set_geom_after_each_caltype) {
                                set_arm_length(cur_set->arm_length, false);
                            }

                        } // caltype.arm_length
                        
                        blink_LED(2);
                        break;

                    case CT_TOWER_ANGLE:
//                        _printf("~ Tower Angle ~\n"); flush();
                        // ****************
                        // * Tower angles *
                        // ****************

                        if(caltype.tower_angle.active) {

                            local_temp = temp * caltype.tower_angle.annealing_temp_mul;

                            for(k=0; k<3; k++) {
                                best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_tower_angle_offsets, cur_set->tower_angle, k, local_temp, test_tower_angle[k].range_min, test_tower_angle[k].range_max, binsearch_width, cur_cartesian, target);
                                move_randomly_towards(cur_set->tower_angle[k], best_value, local_temp, target, overrun_divisor);

                                // Update the robot
                                if(set_geom_after_each_caltype) {
                                    set_tower_angle_offsets(cur_set->tower_angle[X], cur_set->tower_angle[Y], cur_set->tower_angle[Z], false);
                                }

                            } // k

                        } // caltype.tower_angle

                        blink_LED(2);
                        break;

                    case CT_VIRTUAL_SHIMMING:
//                        _printf("~ Shimming ~\n"); flush();
                        // ********************
                        // * Virtual Shimming *
                        // ********************

                        if(caltype.virtual_shimming.active) {

                            local_temp = temp * caltype.virtual_shimming.annealing_temp_mul;

                            for(k=0; k<3; k++) {
                                best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_test_virtual_shimming, cur_set->virtual_shimming, k, local_temp, test_virtual_shimming[k].range_min, test_virtual_shimming[k].range_max, binsearch_width, cur_cartesian, target);
                                move_randomly_towards(cur_set->virtual_shimming[k], best_value, local_temp, target, overrun_divisor);

                                // Update the robot
                                if(set_geom_after_each_caltype) {
                                    set_virtual_shimming(cur_set->virtual_shimming[X], cur_set->virtual_shimming[Y], cur_set->virtual_shimming[Z], false);
                                }

                            } // k

                        } // caltype.virtual_shimming

                        blink_LED(2);
                        break;

                    case CT_TOWER_VECTOR:
//                        _printf("~ Tower Vector ~\n"); flush();
                        blink_LED(2);
                        break;

                }

            }


            // If we haven't been committing the changes to the robot as we've gone through the
            // annealing process, do so now
            if(!set_geom_after_each_caltype) {
                set_trim(cur_set->trim[X], cur_set->trim[Y], cur_set->trim[Z]);
                set_delta_radius(cur_set->delta_radius, false);
                set_tower_radius_offsets(cur_set->tower_radius[X], cur_set->tower_radius[Y], cur_set->tower_radius[Z], false);
                set_arm_length(cur_set->arm_length, false);
                set_tower_angle_offsets(cur_set->tower_angle[X], cur_set->tower_angle[Y], cur_set->tower_angle[Z], false);
                set_virtual_shimming(cur_set->virtual_shimming[X], cur_set->virtual_shimming[Y], cur_set->virtual_shimming[Z], false);
            }


            // Tell the robot to update its position according to the new kinematics
            // Note: This is probably a waste of time. It may have served some better purpose before
            post_adjust_kinematics();


            // *****************************
            // * Re-center all test ranges *
            // *****************************

            test_delta_radius.reset_min_max();
            test_arm_length.reset_min_max();
            for(k=0; k<3; k++) {
                test_endstop[k].reset_min_max();
                test_delta_radius_offset[k].reset_min_max();
                test_tower_angle[k].reset_min_max();
                test_virtual_shimming[k].reset_min_max();
            }
            

            // ****************
            // * Housekeeping *
            // ****************

            if(try_mod_5 == 0) {
                float tempE = simulate_FK_and_get_energy(test_axis, cur_set->trim, cur_cartesian);
                _printf("Try %d of %d, energy=%1.3f (want <= %1.3f)\n", annealing_try, annealing_tries, tempE, global_target);

                // *****************************************************
                // * Keep track of last energy, and abort if it stalls *
                // *****************************************************

                // Shift the last_energy array right by one entry
                for(j = LAST_ENERGY_N - 1; j > 0; j--) {
                    last_energy[j] = last_energy[j - 1];
                }

                // Store the new entry
                last_energy[0] = tempE;

                // The count tells us whether the array is full, and therefore whether it's useful for running statistics
                if(++last_energy_count >= LAST_ENERGY_N) {

                    last_energy_count = LAST_ENERGY_N - 1;

                    // Calc stats
                    float mu, sigma, min, max;
                    calc_statistics(last_energy, LAST_ENERGY_N, mu, sigma, min, max);
                    
                    if(sigma < 0.01) {
                        _printf("Annealing has stalled - aborting.\n");
                        break;
                    }
                    
                    /*
                    // For debugging
                    _printf("Last energy counts: ");
                    for(j=0; j<LAST_ENERGY_N; j++) {
                        _printf("%1.3f ", last_energy[j]);
                    }
                    _printf("sigma=%1.3f\n", sigma);
                    */

                }
                
                // Abort if within the global target
                if(tempE <= global_target) {
                    _printf("Annealing : Within target\n");
                    break;
                }
            } // try_mod_5 == 0

            flush();


        } // annealing_try
        
        float endE = simulate_FK_and_get_energy(test_axis, cur_set->trim, cur_cartesian);
        newline();
        _printf("End of annealing pass (energy=%1.3f)\n", endE);
        
        if(endE <= global_target) {
            _printf("/!\\ SUCCESS /!\\\n");
            break;
        }


        _printf(" \n");

    } // outer_try


    // Print the results
    _printf("Heuristic calibration complete (energy=%1.3f)\n", simulate_FK_and_get_energy(test_axis, cur_set->trim, cur_cartesian));

    // Normalize trim (this prevents downward creep)
    auto mm = std::minmax({ cur_set->trim[X], cur_set->trim[Y], cur_set->trim[Z] });
    cur_set->trim[X] -= mm.second;
    cur_set->trim[Y] -= mm.second;
    cur_set->trim[Z] -= mm.second;
    set_trim(cur_set->trim[X], cur_set->trim[Y], cur_set->trim[Z]);

    print_kinematics();
    
//    newline();
//    _printf("Final SIMULATED depths:\n");
//    print_depths(cur_cartesian);

    if(!simulate_only) {
        _printf("Checking calibration. If it's worse than it was before, you may have to run this several times!\n");
        depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED, false);
        _printf("You can run this command again to see if it gets better, or type M500 to save.\n");
        zprobe->home();
    }

    pop_prefix();

    return true;

}


// Find the most optimal configuration for a test function (e.g. set_delta_radius())
// (float version)
float ComprehensiveDeltaStrategy::find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, bool), float value, float temp, float min, float max, float binsearch_width, float **cartesian, float target) {

    float energy_min, energy_max; 
    int j;

//_printf("find_optimal_config start: value=%1.3f, temp=%1.3f, min=%1.3f, max=%1.3f, binsearch_width=%1.3f\n", value, temp, min, max, binsearch_width);

    // Never search further than we can walk during this iteration
    if(min < value - temp) min = value - temp;
    if(max > value + temp) max = value + temp;

//_printf("find_optimal_config after clipping: min=%1.3f, max=%1.3f\n", min, max);

    // Find the direction of the most optimal configuration using a binary search
    for(j=0; j<250; j++) {

        // Test energy at min & max

        ((this)->*test_function)(min, true);
        energy_min = simulate_FK_and_get_energy(test_axis, cur_set->trim, cartesian);

        ((this)->*test_function)(max, true);
        energy_max = simulate_FK_and_get_energy(test_axis, cur_set->trim, cartesian);

        // Who won?
//_printf("Check: val=%1.3f temp=%1.3f min=%1.3f max=%1.3f\n", value, temp, min, max);
        if(max - min <= target) {
//_printf("Winner!\n");
            break;
        }
        if(energy_min < energy_max) {
//            max -= ((max - min) * binsearch_width);
            max -= binsearch_width;
        }
        if(energy_min > energy_max) {
//            min += ((max - min) * binsearch_width);
            min += binsearch_width;
        }
    
    }

//_printf("Iterations: %d\n", j);
    return (min + max) / 2.0f;

} 


// Find the most optimal configuration for a test function (e.g. set_delta_radius())
// (float[3] version) 
float ComprehensiveDeltaStrategy::find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, float, float, bool), float values[3], int value_idx, float temp, float min, float max, float binsearch_width, float **cartesian, float target) {

    int j;
    float energy_min, energy_max;
    float save_val = values[value_idx];

    // Never search further than we can walk during this iteration
    if(min < save_val - temp) min = save_val - temp;
    if(max > save_val + temp) max = save_val + temp;

    // Find the direction of the most optimal configuration using a binary search
    for(j=0; j<250; j++) {

        // Test energy at min & max
        values[value_idx] = min;
        ((this)->*test_function)(values[X], values[Y], values[Z], true);
        energy_min = simulate_FK_and_get_energy(test_axis, cur_set->trim, cartesian);

        values[value_idx] = max;
        ((this)->*test_function)(values[X], values[Y], values[Z], true);
        energy_max = simulate_FK_and_get_energy(test_axis, cur_set->trim, cartesian);

        // Who won?
//_printf("Check: val=%1.3f temp=%1.3f min=%1.3f max=%1.3f\n", values[value_idx], temp, min, max);
        if(max - min <= target) {
//_printf("Winner!\n");
            break;
        }
        if(energy_min < energy_max) {
//            max -= ((max - min) * binsearch_width);
            max -= binsearch_width;
        }
        if(energy_min > energy_max) {
//            min += ((max - min) * binsearch_width);
            min += binsearch_width;
        }
    
    }

//_printf("Iterations: %d\n", j);
    values[value_idx] = save_val;
    return (min + max) / 2.0f;

} 


// find_optimal_config() requires a test function that takes three floats and returns a bool
bool ComprehensiveDeltaStrategy::set_test_trim(float x, float y, float z, bool dummy) {
    cur_set->trim[X] = x;
    cur_set->trim[Y] = y;
    cur_set->trim[Z] = z;
    return true;
}


bool ComprehensiveDeltaStrategy::set_test_virtual_shimming(float x, float y, float z, bool dummy) {
    cur_set->virtual_shimming[X] = x;
    cur_set->virtual_shimming[Y] = y;
    cur_set->virtual_shimming[Z] = z;
    set_virtual_shimming(x, y, z);
    return true;
}


// Move a random distance in the direction we just figured out in find_optimal_config()
void ComprehensiveDeltaStrategy::move_randomly_towards(float &value, float best, float temp, float target, float overrun_divisor) {

    float step = ( ((float)rand() / RAND_MAX) * temp ) + 0.001;

    if(best > value + target) {
        if(value + step > best) {
            step /= overrun_divisor;
        }
        value += step;
    }
    if(best < value - target) {
        if(value - step < best) {
            step /= overrun_divisor;
        }
        value -= step;
    }

}


// Simulate inverse (cartesian->actuator) kinematics
// cartesian[] will contain the generated test points
// test_axis[] (class member) will contain the generated axis positions
void ComprehensiveDeltaStrategy::simulate_IK(float **cartesian, float trim[3]) {

    float pos[3];

    for(int j = 0; j < DM_GRID_ELEMENTS; j++) {
    
        cartesian[j][X] = test_point[j][X];
        cartesian[j][Y] = test_point[j][Y];

        if(active_point[j] == TP_ACTIVE) {

            // Current cartesian coordinates of the depth map
            cartesian[j][Z] = depth_map[j].rel;
        
            pos[X] = cartesian[j][X];
            pos[Y] = cartesian[j][Y];
            pos[Z] = cartesian[j][Z];
            
            // Adjust Cartesian positions for surface transform plane (virtual shimming)
            if(surface_transform->plane_enabled) {
                pos[Z] += ((-surface_transform->normal[X] * pos[X]) - (surface_transform->normal[Y] * pos[Y]) - surface_transform->d) / surface_transform->normal[Z];
            }
            
            // Query the robot: Where do the axes have to be for the effector to be at these coordinates?
            THEKERNEL->robot->arm_solution->cartesian_to_actuator(pos, test_axis[j]);
        
            // Adjust axis positions to simulate the effects of trim
            test_axis[j][X] += trim[X];
            test_axis[j][Y] += trim[Y];
            test_axis[j][Z] += trim[Z];
        
        } else {
        
            cartesian[j][Z] = 0;
            test_axis[j][X] = 0;
            test_axis[j][Y] = 0;
            test_axis[j][Z] = 0;
        
        }
        
    } // for j

}


// Simulate forward (actuator->cartesian) kinematics (returns the "energy" of the end result)
// The resulting cartesian coordinates are stored in cartesian[][]
//float ComprehensiveDeltaStrategy::simulate_FK_and_get_energy(float axis_position[DM_GRID_ELEMENTS][3], float trim[3], float cartesian[DM_GRID_ELEMENTS][3]) {
float ComprehensiveDeltaStrategy::simulate_FK_and_get_energy(float **axis_position, float trim[3], float **cartesian) {

    float trimmed[3];

    for(int j = 0; j < DM_GRID_ELEMENTS; j++) {

        if(active_point[j] == TP_ACTIVE) {

            trimmed[X] = axis_position[j][X] - trim[X];
            trimmed[Y] = axis_position[j][Y] - trim[Y];
            trimmed[Z] = axis_position[j][Z] - trim[Z];

            THEKERNEL->robot->arm_solution->actuator_to_cartesian(trimmed, cartesian[j]);

            // Adjust Cartesian positions for surface transform plane (virtual shimming)
            if(surface_transform->plane_enabled) {
                cartesian[j][Z] -= ((-surface_transform->normal[X] * cartesian[j][X]) - (surface_transform->normal[Y] * cartesian[j][Y]) - surface_transform->d) / surface_transform->normal[Z];
            }

        }
    }

    return calc_energy(cartesian);

}















// Find test_point[] array index of point closest to coordinates, taking the print surface shape into account
int ComprehensiveDeltaStrategy::find_nearest_test_point(float pos[2]) {

//_printf("FNTP for {%1.3f, %1.3f}\n", pos[X], pos[Y]);

    float dist;
    float lowest = 999;
    int lowest_idx = 0;
    float tp[2];

    for(int i=0; i<DM_GRID_ELEMENTS; i++) {

        tp[X] = test_point[i][X];
        tp[Y] = test_point[i][Y];
        dist = distance2D(pos, tp);
//_printf("Testing point %d: pos={%1.2f, %1.2f} dist=%1.2f, lowest=%1.2f: ", i, tp[X], tp[Y], dist, lowest);

        // FIXME: Are there any conditions where we'd want TP_ACTIVE_NEIGHBOR? Probably not...
        if(active_point[i] == TP_ACTIVE || active_point[i] == TP_CENTER) {
            if(dist < lowest) {
//__printf("Winner so far\n");
                lowest = dist;
                lowest_idx = i;
            } else {
//__printf("Not the winner\n");
            }
        } else {
//__printf("Point not active\n");
        }

    }

    return lowest_idx;

}


// Initialize test points to be used with G31 operations
void ComprehensiveDeltaStrategy::init_test_points() {

    // Initialize "test points" (grid)
    // -----------------------------------------------------
    // The grid's footprint (what it physically measures) is (2 * probe_radius) x (2 * probe_radius) millimeters.
    float x, y;
    int n = 0;
    float point_spacing = (probe_radius * 2) / (DM_GRID_DIMENSION - 1);
    float midpoint_index = (DM_GRID_DIMENSION - 1) / 2; // Index to origin in either dimension (5x5 => 2, 7x7 => 3, etc.)
    _printf("Point spacing: %1.3f\n", point_spacing);
    
    for(y=0; y<DM_GRID_DIMENSION; y++) {
        for(x=0; x<DM_GRID_DIMENSION; x++) {
            
            // Convert from grid coords to Cartesian coords
            float xCart = (-midpoint_index + x) * point_spacing;
            float yCart = (-midpoint_index + y) * -point_spacing; // We have to invert Y

            test_point[n][X] = xCart;
            test_point[n][Y] = yCart;

            // Following two lines must be commented in production to avoid stalling the firmware during init!
//            __printf("Set test point[%d] (X=%1.0f, Y=%1.0f) to { %1.1f, %1.1f }\n", n, x, y, test_point[n][X], test_point[n][Y]);
//            flush();
            
            n++;

        }
    }

/*
//  Old code that works for 5x5, but not 7x7:
    for(y = probe_radius; y >= -probe_radius; y-= point_spacing) {
        for(x = -probe_radius; x <= probe_radius; x += point_spacing) {
            test_point[n][X] = x;
            test_point[n][Y] = y;

            // Following two lines must be commented in production to avoid stalling the firmware during init!
            __printf("Set test point[%d] (X=%1.0f, Y=%1.0f) to { %1.1f, %1.1f }\n", n, x, y, test_point[n][X], test_point[n][Y]);
            flush();

            n++;
        }
    }
*/


    // The method find_nearest_test_point() will only work once the above code is run.


    // Determine active points
    // -----------------------------------------------------
    float origin[2] = { 0, 0 };
    int dm_pos;
    float neighboring_probe_radius = probe_radius + (probe_radius / ((DM_GRID_DIMENSION - 1) / 2));

    //_printf("Probe radius: %1.3f - Neighboring probe radius: %1.3f\n", probe_radius, neighboring_probe_radius);

    // Determine active/inactive points based on print surface shape
    for(y=0; y<DM_GRID_DIMENSION; y++) {

        for(x=0; x<DM_GRID_DIMENSION; x++) {

            // Determine index of this grid position in the depth map array
            dm_pos = (y * DM_GRID_DIMENSION) + x;
    
            switch(surface_shape) {

                // Circle print shape requires determining which points are within probe_radius,
                // and which are their immediate neighbors outside probe_radius
                case PSS_CIRCLE:
                
                    if(distance2D(origin, test_point[dm_pos]) <= probe_radius) {

                        // Within probe radius, and NOT origin: Active
                        active_point[dm_pos] = TP_ACTIVE;

                    } else {

                        // We have to be super picky about what we make a neighbor
                        if(
                          distance2D(origin, test_point[dm_pos]) <= neighboring_probe_radius &&
                          y != 0 &&				// Not on the Y axis
                          y != (DM_GRID_DIMENSION - 1) &&	// Not on the top row
                          y != -(DM_GRID_DIMENSION - 1)) {	// Not on the bottom row

                            // Neighbor
                            active_point[dm_pos] = TP_ACTIVE_NEIGHBOR;

                        } else {

                            // Neither active or neighbor
                            active_point[dm_pos] = TP_INACTIVE;
                        }

                    }
                    break;
                
                // Square print shape is easy: everything is active!
                case PSS_SQUARE:

                    active_point[dm_pos] = TP_ACTIVE;
                    break;

            } // switch
        } // for x
    } // for y

    // Mark the origin point
    active_point[find_nearest_test_point(origin)] = TP_CENTER;

    /*
    // For testing
    for(y=0; y<DM_GRID_DIMENSION; y++) {
        for(x=0; x<DM_GRID_DIMENSION; x++) {
            dm_pos = (y * DM_GRID_DIMENSION) + x;

            __printf("%02d: ", dm_pos);
            switch(active_point[dm_pos]) {
                case TP_CENTER:		 __printf("center   "); break;
                case TP_ACTIVE:          __printf("active   "); break;
                case TP_ACTIVE_NEIGHBOR: __printf("neighbor "); break;
                case TP_INACTIVE:        __printf("inactive "); break;
            }

        }
        newline();
    }
    */


    // Initialize "tower points" (points nearest to a tower)
    // -----------------------------------------------------
    // Towers are 60 degrees off centerline.
    // So, the quadrants look like this:
    // Q2: -xDeg, +yDeg   Q1: +xDeg, +yDeg
    // Q3: -xDeg, -yDeg   Q4: +xDeg, -yDeg
    float xDeg = 0.866025f;
    float yDeg = 0.5;
    float pos[2];

    // Find center
    pos[X] = 0;
    pos[Y] = 0;
    tower_point_idx[TP_CTR] = find_nearest_test_point(pos);

    // Find X tower
    pos[X] = -xDeg * probe_radius;
    pos[Y] = -yDeg * probe_radius;
    tower_point_idx[TP_X] = find_nearest_test_point(pos);

    // Find Y tower
    pos[X] =  xDeg * probe_radius;
    pos[Y] = -yDeg * probe_radius;
    tower_point_idx[TP_Y] = find_nearest_test_point(pos);
    
    // Find Z tower
    pos[X] = 0;
    pos[Y] = probe_radius;
    tower_point_idx[TP_Z] = find_nearest_test_point(pos);

    surface_transform->tri_points[X][X] = test_point[tower_point_idx[TP_X]][X];
    surface_transform->tri_points[X][Y] = test_point[tower_point_idx[TP_X]][Y];
    surface_transform->tri_points[X][Z] = 0;

    surface_transform->tri_points[Y][X] = test_point[tower_point_idx[TP_Y]][X];
    surface_transform->tri_points[Y][Y] = test_point[tower_point_idx[TP_Y]][Y];
    surface_transform->tri_points[Y][Z] = 0;

    surface_transform->tri_points[Z][X] = test_point[tower_point_idx[TP_Z]][X];
    surface_transform->tri_points[Z][Y] = test_point[tower_point_idx[TP_Z]][Y];
    surface_transform->tri_points[Z][Z] = 0;

}


// Set the adjust function. This tells the kernel how to adjust Z for any point.
// I used ThreePointStrategy.cpp as an example.
void ComprehensiveDeltaStrategy::set_adjust_function(bool on) {

    surface_transform->active = on;

    if(on) {
//        _printf("[ST] Depth correction enabled.\n");
        THEKERNEL->robot->compensationTransform = [this](float target[3]) { target[Z] += this->get_adjust_z(target[X], target[Y]); };
    } else {
//        _printf("[ST] Depth correction disabled.\n");
        THEKERNEL->robot->compensationTransform = nullptr;
    }

}


// Figure out how far up or down we need to move the effector to conform to the print surface shape.
// There are two methods here, which can be used in tandem or separately.
// First, we can adjust Z by rotating the virtual plane, a la ThreePointStrategy.cpp.
// Second, we can bilinearly interpolate our coordinates relative to a depth map to approximate the
// correct depth.
//
// Because this is called hundreds of times per second, it has to run FAST. Therefore, stack variables
// are not used (except for whatever temporary ones the compiler uses to do math). Every variable that
// can be set aside beforehand, has been.
float ComprehensiveDeltaStrategy::get_adjust_z(float targetX, float targetY) {

    bili->st_z_offset = 0;

    // Adjust Z according to the rotation of the plane of the print surface
    if(surface_transform->plane_enabled && surface_transform->active) {

        bili->st_z_offset = ((-surface_transform->normal[X] * targetX) - (surface_transform->normal[Y] * targetY) - surface_transform->d) / surface_transform->normal[Z];

    }

    // Adjust Z according to depth map
    if(surface_transform->depth_enabled && surface_transform->active) {


        // Based on code retrieved from:
        // http://stackoverflow.com/questions/8808996/bilinear-interpolation-to-enlarge-bitmap-images

        // Determine which quad the point is in
        // Thx Liviu:
        // http://stackoverflow.com/questions/16592870/map-the-point-x-y-in-cartesian-coordinate-into-2d-array-elements
        // ----------------------------------------------------------------------
        // The print surface is in Cartesian.
        // Our array is in single-quadrant (origin at 0,0; X grows right and Y grows down).
        // Translate surface coordinates to array coordinates by adding the difference between coordinate systems.

        // Constrain data and calculate array positions & bounding box
        // ----------------------------------------------------------------------
        // Constrain tested points to probe radius

//static int count = 0;
//count = 0;
//if(count % 100 == 0 && zdebug) {
//    _printf("targetX=%1.3f targetY=%1.3f\n", targetX, targetY);
//    _printf("probe_radius=%1.3f scaler=%1.3f\n", probe_radius, bili->cartesian_to_array_scaler);
//
//}

        targetX = clamp(targetX, -probe_radius, probe_radius);
        targetY = clamp(targetY, -probe_radius, probe_radius);

        // Calculate (floating-point) array position
        bili->array_x = (targetX - -probe_radius) * bili->cartesian_to_array_scaler;
        bili->array_y = (-targetY - -probe_radius) * bili->cartesian_to_array_scaler;	// Y inverted since it starts high and ends low

        // Calculate bounding box
        bili->x1 = floor(bili->array_x);
        bili->y1 = floor(bili->array_y);
        bili->x2 = bili->x1 + 1;
        bili->y2 = bili->y1 + 1;
//if(count % 100 == 0 && zdebug) {
//    _printf("array_x=%1.3f array_y=%1.3f\n", bili->array_x, bili->array_y);
//    _printf("Bounding box: {%1.1f, %1.1f} to {%1.1f, %1.1f}\n", bili->x1, bili->y1, bili->x2, bili->y2);
//}


        // Calculate surface transform array indices for bounding box corners
        // ----------------------------------------------------------------------
        //  x1 ____________ x2  
        // y1 | Q11    Q21
        //    | 
        //    |
        // y2 | Q12    Q22
        bili->st_Q11 = (bili->y1 * DM_GRID_DIMENSION) + bili->x1;
        bili->st_Q12 = (bili->y2 * DM_GRID_DIMENSION) + bili->x1;
        bili->st_Q21 = (bili->y1 * DM_GRID_DIMENSION) + bili->x2;
        bili->st_Q22 = (bili->y2 * DM_GRID_DIMENSION) + bili->x2;

        // Retrieve heights from the quad's points
        // ----------------------------------------------------------------------
        bili->Q11 = surface_transform->depth[bili->st_Q11];
        bili->Q12 = surface_transform->depth[bili->st_Q12];
        bili->Q21 = surface_transform->depth[bili->st_Q21];
        bili->Q22 = surface_transform->depth[bili->st_Q22];

        // Set up the first terms
        // ----------------------------------------------------------------------
        bili->divisor = (bili->x2 - bili->x1) * (bili->y2 - bili->y1);
        bili->first_term[0] = bili->Q11 / bili->divisor;
        bili->first_term[1] = bili->Q21 / bili->divisor;
        bili->first_term[2] = bili->Q12 / bili->divisor;
        bili->first_term[3] = bili->Q22 / bili->divisor;

        // Set up the second and third terms
        // ----------------------------------------------------------------------
        bili->x2_minus_x = bili->x2 - bili->array_x;
        bili->x_minus_x1 = bili->array_x - bili->x1;
        bili->y2_minus_y = bili->y2 - bili->array_y;
        bili->y_minus_y1 = bili->array_y - bili->y1;

//if(count % 100 == 0 && zdebug) {
//    _printf("Indices for array entries for this BB: Q11=%d Q21=%d Q12=%d Q22=%d\n", bili->st_Q11, bili->st_Q21, bili->st_Q12, bili->st_Q22);
//    _printf("Heights: {%1.2f, %1.2f, %1.2f, %1.2f}\n", bili->Q11, bili->Q12, bili->Q21, bili->Q22);
//    _printf("First terms: %1.2f, %1.2f, %1.2f, %1.2f\n", bili->first_term[0], bili->first_term[1], bili->first_term[2], bili->first_term[3]);
//    _printf("Second & third terms: x2_minus_x=%1.2f; x_minus_x1=%1.2f; y2_minus_y=%1.2f; y_minus_y1=%1.2f\n", bili->x2_minus_x, bili->x_minus_x1, bili->y2_minus_y, bili->y_minus_y1);
//}

        // Interpolate    
        // ----------------------------------------------------------------------
        bili->result =
            bili->first_term[0] * bili->x2_minus_x * bili->y2_minus_y +
            bili->first_term[1] * bili->x_minus_x1 * bili->y2_minus_y +
            bili->first_term[2] * bili->x2_minus_x * bili->y_minus_y1 +
            bili->first_term[3] * bili->x_minus_x1 * bili->y_minus_y1;

        bili->st_z_offset += bili->result;

//if(count % 100 == 0 && zdebug) {
//    _printf("st_z_offset = %1.3f\n", st_z_offset);
//}
//
//count++;

    }

    /*
    static int count = 0;
    if(count % 1000 == 0) {
        __printf("returning offset %1.3f\n", st_z_offset);
    }
    count++;
    */
    
    return bili->st_z_offset;

}



// Measure probe tolerance (repeatability)
// Things that may have an impact on repeatability:
// - How tightly the probe is printed and/or built
// - Controller cooling, especially the stepper drivers
// - Noise from other wiring in the chassis
// - feedrate
// - debounce_count
// - probe_smoothing
bool ComprehensiveDeltaStrategy::measure_probe_repeatability(Gcode *gcode) {

    // Statistical variables
    int i;
    int steps;
    int nSamples = 10;
    float mu = 0;	// Mean
    float sigma = 0;	// Standard deviation
    float dev = 0;	// Sample deviation

    push_prefix("PR");

    // Options
    float want_acceleration = probe_acceleration;
    bool do_eccentricity_test = false;

    // Process G-code params, if any
    if(gcode != nullptr) {
        if(gcode->has_letter('A')) {
            want_acceleration = gcode->get_value('A');
            if(want_acceleration < 1 || want_acceleration > 1000) {
                want_acceleration = probe_acceleration;
            }
        }
        if(gcode->has_letter('B')) {
            int db = (int)gcode->get_value('B');
            if(db <    0) db = 0;
            if(db > 2000) db = 2000;
            zprobe->setDebounceCount(db);
        }
        if(gcode->has_letter('D')) {
            // This will be cast to a bool
            zprobe->setDecelerateOnTrigger((bool)gcode->get_value('D'));
        }
        if(gcode->has_letter('E')) {
            do_eccentricity_test = true;
        }
        if(gcode->has_letter('P')) {
            probe_smoothing = (unsigned int)gcode->get_value('P');
            if(probe_smoothing <  0) probe_smoothing = 0;
            if(probe_smoothing > 10) probe_smoothing = 10;
        }
        if(gcode->has_letter('Q')) {
            probe_priming = (unsigned int)gcode->get_value('Q');
            // If your probe takes more than 20 hits to settle, you should figure out why :(
            if(probe_priming <  0) probe_priming = 0;
            if(probe_priming > 20) probe_priming = 20;
        }
        if(gcode->has_letter('U')) {
            // ZProbe sanity-checks this already
            zprobe->setFastFeedrate(gcode->get_value('U'));
        }
        if(gcode->has_letter('V')) {
            // ZProbe sanity-checks this already
            zprobe->setSlowFeedrate(gcode->get_value('V'));
        }
        if(gcode->has_letter('S')) {
            nSamples = (int)gcode->get_value('S');
            if(nSamples > 30) {
                _printf("Too many samples!\n");
                pop_prefix();
                return false;
            }
        }
    }

    float sample[nSamples];
    if(probe_smoothing < 1) probe_smoothing = 1;
    if(probe_smoothing > 10) probe_smoothing = 10;

    // Print settings
    _printf("   Repeatability test: %d samples (S)\n", nSamples);
    _printf("     Acceleration (A): %1.1f\n", want_acceleration = 0 ? THEKERNEL->planner->get_acceleration() : want_acceleration);
    _printf("   Debounce count (B): %d\n", zprobe->getDebounceCount());
    _printf(" Smooth decel (D0|D1): %s\n", zprobe->getDecelerateOnTrigger() ? _STR_TRUE_ : _STR_FALSE_);
    _printf("Eccentricity test (E): %s\n", do_eccentricity_test ? _STR_ON_ : _STR_OFF_);
    _printf("  Probe smoothing (P): %d\n", probe_smoothing);
    _printf("    Probe priming (Q): %d\n", probe_priming);
    _printf("            Feedrates: Fast (U) = %1.3f, Slow (V) = %1.3f\n", zprobe->getFastFeedrate(), zprobe->getSlowFeedrate());
    _printf("1 step = %1.5f mm.\n", zprobe->zsteps_to_mm(1.0f));
 
    // Move into position, after safely determining the true bed height
    prepare_to_probe();

//    // Prime the probe (run it a number of times to get it to "settle")
//    if(!prime_probe()) {
//        pop_prefix();
//        return false;
//    }

    float xDeg = 0.866025f;
    float yDeg = 0.5f;
    float radius = 10;// probe_radius;

    // Move the probe around to see if we can throw it off (e.g.: if it's loose, the printer has "delta arm blues", etc.)
    for(i=0; i<nSamples; i++) {

        if(do_eccentricity_test) {

            // Move towards X
            zprobe->coordinated_move(-xDeg * radius, -yDeg * radius, NAN, zprobe->getFastFeedrate(), false);
            zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate(), false);
                
            // Move towards Y
            zprobe->coordinated_move(xDeg * radius, -yDeg * radius, NAN, zprobe->getFastFeedrate(), false);
            zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate(), false);
                
            // Move towards Z
            zprobe->coordinated_move(0, radius, NAN, zprobe->getFastFeedrate(), false);
            zprobe->coordinated_move(0, 0, NAN, zprobe->getFastFeedrate(), false);

        }

        // Probe at center
        if(do_probe_at(steps, 0, 0)) {
            sample[i] = steps;
            _printf("Test %2d of %2d: Measured %d steps (%1.3f mm)\n", i + 1, nSamples, steps, zprobe->zsteps_to_mm(steps));
            if(steps > 50000) {
                _printf("Discarding result and trying again. Check probe_height.\n");
                i--;
            } else {
                mu += (float)steps;
            }
        } else {
            _printf("do_probe_at() returned false. Check probe_height.\n");
            pop_prefix();
            return false;
        }
    }
            
    // Mean
    mu /= nSamples;
            
    // Range and standard deviation
    int min=9999, max=0;
    for(i=0; i<nSamples; i++) {
        dev += powf((float)sample[i] - mu, 2);
        if(sample[i] < min) min = sample[i];
        if(sample[i] > max) max = sample[i];
    }
    sigma = sqrtf(dev/nSamples);

    // I dare anyone to tell me this should be an interquartile mean...
    float rep = zprobe->zsteps_to_mm(max - min);

    // Print stats
    _printf("Stats:\n");
    _printf("  range: %d steps (%1.4f mm)\n", max - min, zprobe->zsteps_to_mm(max - min));
    _printf("     mu: %1.3f steps (%1.3f mm)\n", mu, zprobe->zsteps_to_mm(mu));
    _printf("  sigma: %1.3f steps (%1.3f mm)\n", sigma, zprobe->zsteps_to_mm(sigma));
    _printf("Repeatability: %1.4f (add a little to be sure)\n", rep);

    if(best_probe_calibration->sigma == -1 || sigma < best_probe_calibration->sigma) {

        _printf("This is your best score so far!\n");
        best_probe_calibration->sigma = sigma;
        best_probe_calibration->range = max - min;
        best_probe_calibration->accel = want_acceleration;
        best_probe_calibration->debounce_count = zprobe->getDebounceCount();
        best_probe_calibration->decelerate = zprobe->getDecelerateOnTrigger();
        best_probe_calibration->eccentricity = do_eccentricity_test;
        best_probe_calibration->smoothing = probe_smoothing;
        best_probe_calibration->priming = probe_priming;
        best_probe_calibration->fast = zprobe->getFastFeedrate();
        best_probe_calibration->slow = zprobe->getSlowFeedrate();

    } else {

        _printf(
            "Best score so far: [sigma=%1.3f, range=%d] => accel=%f, debounce=%d, decelerate=%s, eccentricity=%s, smoothing=%d, priming=%d, fastFR=%1.3f, slowFR=%1.3f\n",
            best_probe_calibration->sigma,
            best_probe_calibration->range,
            best_probe_calibration->accel,
            best_probe_calibration->debounce_count,
            best_probe_calibration->decelerate ? _STR_TRUE_ : _STR_FALSE_,
            best_probe_calibration->eccentricity ? _STR_ON_ : _STR_OFF_,
            best_probe_calibration->smoothing,
            best_probe_calibration->priming,
            best_probe_calibration->fast,
            best_probe_calibration->slow
        );

    }

    // Print evaluation
    _printf("This score is ");
    if(rep < 0.015) {
        __printf("very good!");
    } else if(rep <= 0.03) {
        __printf("average.");
    } else if(rep <= 0.04) {
        __printf("borderline.");
    } else {
        __printf("HORRIBLE.");
    }
    newline();
    newline();

    pop_prefix();
    return true;

}



// Depth-map the print surface
// Initially useful for diagnostics, but the data may be useful for doing live height corrections
// Depths are stored in depth_map (class member)
bool ComprehensiveDeltaStrategy::depth_map_print_surface(float **cartesian, _cds_dmps_result display_results, bool extrapolate_neighbors) {

/*

    Probe-to-edge strategy
    
    PROBLEM:	With a 5x5 (or even 7x7) grid, selecting sample points based on whether they're within PROBE_RADIUS
                results in a diamond-shaped probing area that omits a lot of the periphery. This is no good!
    
    SOLUTION:	We have allocated memory for points outside the circle, so we can use them - we just can't probe them
                at their coordinates because they lie outside probe_radius. However, we CAN probe as close to it as
                possible and use that to interpolate the right value for it:

                * = test point (inside or outside probe_radius)
                / = edge of probe_radius

                Out  A     In
                *    /     *

                1: Sample at point A
                2: Calculate slope between In and A
                3: Project depth at Out based on that slope and its distance from In
                4: Set that depth as Out's depth
                
                Therefore, when the probe returns to point A in the future (with depth correction enabled), it will
                be at the same depth it determined before. We don't have to do anything to tell the interpolation
                routine, get_adjust_z(), because it already measures those points.

                Loading and saving will work in exactly the same way, as well.
    
                We should store the touch points for TP_X and TP_Y separately so that we can use them for the
                iterative calibration routine, and for adjusting the plane surface normal.

*/


    push_prefix("DM");

    static int origin_steps=0;	// Steps from probe_height to bed surface at bed center (static because we don't always probe it!)
    int steps=0; 		// Steps from probe_height to bed surface at one of the test points
    float center[2] = {0, 0};
    int center_point = find_nearest_test_point(center);
    bool force_print = false;

    // Check bed temp to see whether we need to abort, or force a re-probe
    float temp, target;
    static float last_temp=-100, last_target=-100;

    if(!get_bed_temp(temp, target)) {
        return false;
    }

    /* Test code
    geom_dirty = false;
    _printf(" --  geom_dirty=%d\n", geom_dirty);
    _printf(" --  temp=%1.3f, target=%1.3f\n", temp, target);
    _printf(" --  Last temp=%1.3f, last target=%1.3f.\n", last_temp, last_target);
    _printf(" --  fabs(target - temp) = %1.3f\n", fabs(target - temp));
    _printf(" --  fabs(temp - last_temp) = %1.3f\n", fabs(temp - last_temp));
    */

    if(last_temp==-100) {
        last_temp = temp;
        last_target = target;
    }

    // Target set, but not there yet? Ask them to wait & come back.
    if(target != 0 && fabs(target - temp) > 0.5) {
        _printf("/!\\ Heated bed has not stabilized at target temp. Please wait until it is, and then re-run this command.\n");
        geom_dirty = true;
        return false;
    }
    //_printf("Got past the fabs(target-temp) > 0.5 check.\n");

    // Clean geometry and within half a degree of target, but target is different from what it was before? (Re-)probe.
    if( !geom_dirty && (fabs(temp - last_temp) > 1 || target != last_target) ) {
        _printf("/!\\ Bed temp or target temp changed - forcing re-probe.\n");
        geom_dirty = true;
    }
    //_printf("Got past the geom_dirty, fabs(temp - last_temp) > 1, and target != last_target checks.\n");
    //_printf("Passed with geom_dirty = %d.\n", geom_dirty);
    
    last_temp = temp;
    last_target = target;


    // If geometry is clean, just copy the last results over rather than wasting time on re-probing the same numbers
    if(!geom_dirty) {

        _printf("Geometry hasn't changed since last depth map - keeping it!\n \n");
        force_print = true;
        
        // Copy the last results out of depth_map
        for(int i=0; i<DM_GRID_ELEMENTS; i++) {
            cartesian[i][X] = test_point[i][X];
            cartesian[i][Y] = test_point[i][Y];
            cartesian[i][Z] = depth_map[i].rel;
        }

    // If geometry is dirty, (re-)probe.
    } else {

        // Geometry is dirty - we must re-probe the print surface        
        //_printf("Index of center point is %d\n", center_point);

        // Measure depth from probe_from_height at bed center
        prepare_to_probe();

        if(do_probe_at(origin_steps, 0, 0)) {

            depth_map[center_point].rel = 0;
            depth_map[center_point].abs = zprobe->zsteps_to_mm(origin_steps);
            cartesian[center_point][X] = test_point[center_point][X];
            cartesian[center_point][Y] = test_point[center_point][Y];
            cartesian[center_point][Z] = depth_map[center_point].rel;

            if(display_results != RESULTS_NONE) {
                _printf("Depth to bed surface at center: %d steps (%1.3f mm)\n", origin_steps, zprobe->zsteps_to_mm(origin_steps));
                newline();
            }

        } else {

            _printf("Couldn't measure depth to origin.\n");
            pop_prefix();
            return false;

        }

        //origin_steps = 700;

        // Measure depth from probe_height at all test points
        float best = 999;
        float worst = 0;

        // FIRST PASS: Depth-map all active points
        uint8_t row = 0;
        for(int i=0; i<DM_GRID_ELEMENTS; i++) {

            // If active_points_only, we only probe the points figured out in init_test_points(); else we probe them all
            // We don't probe TP_CTR because we already did above, in order to be able to store relative depths
            if(active_point[i] == TP_ACTIVE) {

                // Run the probe
                if(!do_probe_at(steps, test_point[i][X], test_point[i][Y])) {
                    _printf("do_probe_at() returned false.\n");
                    pop_prefix();
                    return false;
                }
                //steps = 600;

                // Store result in depth_map
                depth_map[i].rel = zprobe->zsteps_to_mm(origin_steps - steps);
                depth_map[i].abs = zprobe->zsteps_to_mm(steps);

                // ...And in cartesian[]
                // FIXME: I think there is a redundancy here... need to see how both arrays are used by callers.
                cartesian[i][X] = test_point[i][X];
                cartesian[i][Y] = test_point[i][Y];
                cartesian[i][Z] = depth_map[i].rel;

                // Do some statistics (sign doesn't matter, only magnitude)
                if(fabs(depth_map[i].rel) < fabs(best)) {
                    best = fabs(depth_map[i].rel);
                }

                if(fabs(depth_map[i].rel) > fabs(worst)) {
                    worst = fabs(depth_map[i].rel);
                }

                if(display_results == RESULTS_UNFORMATTED) {

                    // We're going to plainly print the results, one by one, with no special formatting
                    _printf("Depth: %1.3fmm (%1.3fmm absolute)\n", depth_map[i].rel, depth_map[i].abs);

                }

            }

            // Line-by-line output
            if(display_results == RESULTS_FORMATTED && i != 0 && i % DM_GRID_DIMENSION == 0) {
                print_depths_line(depth_map, row++, false);
            }

            flush();

        }

        // Print final line
        if(display_results == RESULTS_FORMATTED) {
            print_depths_line(depth_map, row, false);
        }
        
    } // if(geom_dirty)

    
    // SECOND PASS: Probe neighboring-active points and interpolate.
    // We always do this, even if the geometry isn't dirty, because it isn't probed by default.
    // We're doing two loops because it would have been a hassle to make one loop do everything.
    // The points are probed in array order, and the active-neigbhor points on the left can't be computed
    // until their within-radius neighbors' heights are known.
    if(extrapolate_neighbors) {

        _printf("Probing at circle's edge for extrapolation...\n");

        prepare_to_probe();

        for(int i=0; i<DM_GRID_ELEMENTS; i++) {

            if(active_point[i] == TP_ACTIVE_NEIGHBOR) {

                float coords[2];
                int active_idx;

                // X is the coordinate at print_radius.
                // Equation - complete the squares: x^2 + y^2 = probe_radius^2 - solve for x.
                // ...
                // x^2 = probe_radius^2 - y^2
                // x = sqrt(probe_radius^2 - y^2)
                coords[X] = sqrt( (probe_radius * probe_radius) - (test_point[i][Y] * test_point[i][Y]) );

                // Necessary to flip coords in Q2/3 because the sqrt(... code above only produces positive results.
                // Technically, the equation produces "two" answers because by definition, there are TWO X coords
                // for any given Y - one on the left side of the circle, and the other on the right side.
                if(test_point[i][X] > 0) {
                    active_idx = i - 1;	// Neighboring point is to the left
                } else {
                    active_idx = i + 1;	// Neighboring point is to the right
                    coords[X] = -coords[X];
                }
                
                // Y coordinate is the same whether active or active-neighbor
                coords[Y] = test_point[i][Y];

                // Run the probe
                if(!do_probe_at(steps, coords[X], coords[Y])) {
                    _printf("do_probe_at() returned false.\n");
                    pop_prefix();
                    return false;
                }
                //_printf("Measured %d steps (%1.3f mm).\n", steps, zprobe->zsteps_to_mm(steps));
                // steps = 500;

                // To extrapolate, we need the depths of the active-neighbor, and its associated active point
                struct point_type {
                    float x;
                    float y;
                    cds_depths_t z;
                };

                // Extrapolate depth at test_point[i] based on the slope between the depths of the active test point & probed point
                point_type active { test_point[active_idx][X], test_point[active_idx][Y], { depth_map[active_idx].abs, depth_map[active_idx].rel } };
                point_type probed { coords[X], coords[Y], { zprobe->zsteps_to_mm(steps), zprobe->zsteps_to_mm(origin_steps - steps) } };
                point_type extrap { test_point[i][X], test_point[i][Y], { 0, 0 } };

                //_printf("neighbor=%d, active=%d\n", i, active_idx);
                //_printf("active = {%1.3f, %1.3f, %1.3f | %1.3f}\n", active.x, active.y, active.z.abs, active.z.rel);
                //_printf("probed = {%1.3f, %1.3f, %1.3f | %1.3f}\n", probed.x, probed.y, probed.z.abs, probed.z.rel);
                //_printf("...\n");

                cds_depths_t rise { probed.z.abs - active.z.abs, probed.z.rel - active.z.rel };
                float dist_active_to_extrap = sqrt(pow(extrap.x - active.x, 2));
                float dist_active_to_probed = sqrt(pow(probed.x - active.x, 2));
                float dist_mul = dist_active_to_extrap / dist_active_to_probed; // This will be 1.something

                //_printf("rise = %1.3f | %1.3f\n", rise.abs, rise.rel);
                //_printf("dist active to extrap = %1.3f\n", dist_active_to_extrap);
                //_printf("dist active to probed = %1.3f\n", dist_active_to_probed);
                //_printf("dist mul = %1.3f\n", dist_mul);

                extrap.z.abs = active.z.abs + (rise.abs * dist_mul);
                extrap.z.rel = zprobe->zsteps_to_mm(origin_steps) - extrap.z.abs;

                //_printf("extrap = {%1.3f, %1.3f, %1.3f | %1.3f}\n", extrap.x, extrap.y, extrap.z.abs, extrap.z.rel);
                //newline();

                // Store result in depth_map
                depth_map[i].rel = extrap.z.rel;
                depth_map[i].abs = extrap.z.abs;

                // ...And in cartesian[]
                // FIXME: I think there is a redundancy here... need to see how both arrays are used by callers.
                cartesian[i][X] = test_point[i][X];
                cartesian[i][Y] = test_point[i][Y];
                cartesian[i][Z] = depth_map[i].rel;
                //_printf("Set relative depth to %1.3f (%d steps)\n", depth_map[i].rel, steps);

            } // if TP_ACTIVE_NEIGHBOR

        } // for i

    } else {

        for(int i=0; i<DM_GRID_ELEMENTS; i++) {

            if(active_point[i] == TP_ACTIVE_NEIGHBOR) {

                depth_map[i].abs = 0;
                depth_map[i].rel = 0;
                cartesian[i][X] = test_point[i][X];
                cartesian[i][Y] = test_point[i][Y];
                cartesian[i][Z] = 0;

            }

        }

    } // if(extrapolate_neighbors)


    // --------------------
    // End of depth mapping
    // --------------------


    // Show the results (pretty)
    if(display_results == RESULTS_FORMATTED) {

        if(extrapolate_neighbors) {

            newline();
            _printf("Complete output, with extrapolated points (numbers in brackets):\n");
            print_depths(depth_map, true);

        } else if(force_print) {

            print_depths(depth_map, false);

        }

        print_depths_statistics(depth_map);

    }

    pop_prefix();
    
    geom_dirty = false;
    
    return true;

}


// Perform a GeneB-style calibration on the endstops and delta radius.
// Unlike GeneB's method, this converges both at the same time and should produce a slightly better calibration.
// It's a good idea to run this before the heuristic calibration, so it has a good starting point.
bool ComprehensiveDeltaStrategy::iterative_calibration(bool keep_settings) {

    push_prefix("IC");
    print_task_with_warning("Iterative calibration");

    zero_depth_maps();
    set_adjust_function(false);		// Surface plane can confound this method
    
    if(keep_settings) {
        _printf("Keeping kinematics.\n");
    } else {
        _printf("Resetting kinematics.\n");
        set_trim(0, 0, 0);
        set_tower_radius_offsets(0, 0, 0);
        set_tower_angle_offsets(0, 0, 0);
        set_tower_arm_offsets(0, 0, 0);
        set_virtual_shimming(0, 0, 0);
    }

    print_kinematics();

    // Init test points specific to this routine (we don't use the grid)
    // -----------------------------------------------------------------
    // Towers are 60 degrees off centerline.
    // So, the quadrants look like this:
    // Q2: -xDeg, +yDeg   Q1: +xDeg, +yDeg
    // Q3: -xDeg, -yDeg   Q4: +xDeg, -yDeg
    float xDeg = 0.866025f;
    float yDeg = 0.5;
    float tower[3][2];	// [tower][xy]

    // X tower
    tower[X][X] = -xDeg * probe_radius;
    tower[X][Y] = -yDeg * probe_radius;

    // Y tower
    tower[Y][X] =  xDeg * probe_radius;
    tower[Y][Y] = -yDeg * probe_radius;
    
    // Z tower
    tower[Z][X] = 0;
    tower[Z][Y] = probe_radius;

    // Different calibration types can be turned on and off
    // For now we only do endstops and delta radius, but other types can be added as well
    caltype.endstop.active = true;
    caltype.delta_radius.active = true;

    // This is the target accuracy. 30 microns is pretty good.
    float target = 0.03;

    // Steps from probe height to trigger
    int steps;

    // Indexed by TP_CTR|X|Y|Z
    float depth[4];

    // Main loop
    for(int outer_i = 0; outer_i < 20; outer_i++) {

        // Banner preceded by line break for easy visual parsing
        newline();
        _printf("Iteration %d (max %d)\n", outer_i + 1, 20);
    
        // Determine center height
        prepare_to_probe();

//        if(!prime_probe()) {
//            pop_prefix();
//            return false;
//        }

        if(do_probe_at(steps, 0, 0)) {
            depth[TP_CTR] = zprobe->zsteps_to_mm(steps);
        } else {
            pop_prefix();
            return false;
        }
        
        // Determine depth near each tower
        if(!do_probe_at(steps, tower[X][X], tower[X][Y])) {
            pop_prefix();
            return false;
        }
        depth[TP_X] = zprobe->zsteps_to_mm(steps);

        if(!do_probe_at(steps, tower[Y][X], tower[Y][Y])) {
            pop_prefix();
            return false;
        }
        depth[TP_Y] = zprobe->zsteps_to_mm(steps);

        if(!do_probe_at(steps, tower[Z][X], tower[Z][Y])) {
            pop_prefix();
            return false;
        }
        depth[TP_Z] = zprobe->zsteps_to_mm(steps);

        // Deviation for towers
        // These are measured for all calibration types
        auto tower_minmax = std::minmax({ depth[TP_CTR], depth[TP_X], depth[TP_Y], depth[TP_Z] });
        float tower_deviation = tower_minmax.second - tower_minmax.first;

        // Do we calibrate the endstops?
        if(caltype.endstop.active) {

            // ****************
            // *** ENDSTOPS ***
            // ****************
            
            push_prefix("ES");
            
            // Deviation and trimscale
            static float last_deviation;
            static float trimscale;
                
            // Do we need to reset the variables?
            if(caltype.endstop.needs_reset) {
                last_deviation = 999;
                trimscale = 1.3F;
                caltype.endstop.needs_reset = false;
            }

            _printf("Endstops: Difference => %1.3f (want %1.3f)", tower_deviation, target);

            // Deviation within tolerance?
            if(fabs(tower_deviation) <= target) {
                
                // Yep
                newline();
                _printf("Endstops are within tolerance.\n");
                caltype.endstop.in_tolerance = true;
                    
            } else {
                
                // Nope
                __printf(", out of tolerance by %1.3f.\n", tower_deviation - target);
                caltype.endstop.in_tolerance = false;
                    
                // Get trim
                float trim[3];
                if(!get_trim(trim[X], trim[Y], trim[Z])) {
                    _printf("Couldn't query trim.\n");
                    pop_prefix();
                    return false;
                }
                    
                // Sanity-check the trim
                if(trim[X] > 0) trim[X] = 0;
                if(trim[Y] > 0) trim[Y] = 0;
                if(trim[Z] > 0) trim[Z] = 0;
                
                if(trim[X] < -5 || trim[Y] < -5 || trim[Z] < -5) {
                    _printf("Trim: {%1.3f, %1.3f, %1.3f}\n", trim[X], trim[Y], trim[Z]);
                    _printf("Values less than -5 suggest that something is horribly wrong.\n");
                    pop_prefix();
                    return false;
                }
                    
                // If things stayed the same or got worse, we reduce the trimscale
                if((tower_deviation >= last_deviation) && (trimscale * 0.95 >= 0.9)) {  
                    trimscale *= 0.9;
                    _printf("/!\\ Deviation same or worse vs. last time - reducing trim scale to %1.3f\n", trimscale);
                }
                last_deviation = tower_deviation;
                    
                // Set all towers' trims
                trim[X] += (tower_minmax.first - depth[TP_X]) * trimscale;
                trim[Y] += (tower_minmax.first - depth[TP_Y]) * trimscale;
                trim[Z] += (tower_minmax.first - depth[TP_Z]) * trimscale;
                    
                // Correct the downward creep issue by normalizing the trim offsets
                auto mm = std::minmax({trim[X], trim[Y], trim[Z]});
                trim[X] -= mm.second;
                trim[Y] -= mm.second;
                trim[Z] -= mm.second;
                _printf("Setting endstops to {%1.3f, %1.3f, %1.3f}.\n", trim[X], trim[Y], trim[Z]);
                    
                set_trim(trim[X], trim[Y], trim[Z]);
                    
            }
            
            pop_prefix();

        } // caltype.endstop.active
        
        
        if(caltype.delta_radius.active) {

            // ********************                
            // *** DELTA RADIUS ***
            // ********************
            
            push_prefix("DR");
            
            float dr_factor = 2.0;
                
            // Retrieve delta radius or die trying
            float delta_radius;
            if(!get_delta_radius(delta_radius)) {
                _printf("Couldn't query delta_radius.\n");
                pop_prefix();
                return false;
            }

            // Examine differences between tower depths and use this to adjust delta_radius
            float avg = (depth[TP_X] + depth[TP_Y] + depth[TP_Z]) / 3.0;
            float deviation = depth[TP_CTR] - avg;
            _printf("Delta Radius - Depths: Center=%1.3f, Tower average=%1.3f => Difference: %1.3f (want %1.3f), ", depth[TP_CTR], avg, deviation, target);

            // Deviation within tolerance?
            if(fabs(deviation) <= target) {

                // Yep
                __printf("within tolerance.\n");
                caltype.delta_radius.in_tolerance = true;

            } else {

                // Nope
                __printf("out of tolerance by %1.3f.\n", deviation - target);
                caltype.delta_radius.in_tolerance = false;
                    
                _printf("Changing delta radius from %1.3f to ", delta_radius);
                delta_radius += (deviation * dr_factor);
                __printf("%1.3f\n", delta_radius);
                set_delta_radius(delta_radius);

            }

            pop_prefix();

        } // caltype.delta_radius.active


        // Done with ALL tasks?
        // Right now this only does the endstops & delta radius, but more can be added later.
        if(caltype.endstop.in_tolerance && caltype.delta_radius.in_tolerance) {
            print_kinematics();
            _printf("All done! Save settings with M500.\n");
            pop_prefix();
            zprobe->home();
            return true;
        }


    } // for outer_i

    _printf("Maximum tries exceeded. If this is good enough, type M500 to save.\n");
    pop_prefix();
    return true;

}


// Prepare to probe
void ComprehensiveDeltaStrategy::prepare_to_probe() {

    // Determine bed_height, probe_from_height, and probe_height_to_trigger
    if(probe_from_height == -1) {
        find_bed_center_height();
    }

    // Home the machine
    zprobe->home();

    // Do a relative move to an depth of probe_height
    zprobe->coordinated_move(NAN, NAN, -probe_from_height, zprobe->getFastFeedrate(), true);

}


/*
// Enforce clean geometry
bool ComprehensiveDeltaStrategy::require_clean_geometry() {

    if(geom_dirty) {
        __printf("[EC] Geometry has been changed - recalibrating.\n");
        if(!iterative_calibration(false)) return false;
        if(!find_bed_center_height(true)) return false;		// Reset probe_from_height, since the endstop trim may have been changed
        geom_dirty = false;
    }

    return true;

}
*/

// Prime the probe, if set
bool ComprehensiveDeltaStrategy::prime_probe() {

    if(probe_priming) {
        int i, steps=0;
//        __printf("[PR] Priming probe %d times.\n", probe_priming);
        for(i=0; i<probe_priming; i++) {
            if(!run_probe(steps, true, true)) {
                return false;
            }
        }
    }
    return true;

}


// Probe the center of the bed to determine its height in steps, taking probe offsets into account.
// Refreshes the following variables, AND SHOULD BE CALLED BEFORE READING THEM:
//	bed_height
//	probe_from_height
// 	mm_probe_height_to_trigger
bool ComprehensiveDeltaStrategy::find_bed_center_height(bool reset_all) {

    push_prefix("BH");

    // Step counter
    int steps;

    // Start from the top
    zprobe->home();

    // Did they ask for a complete reset? (This means we have to re-find bed center height)
    if(reset_all) {
        probe_from_height = -1;
    }
 
    // If we haven't determined the probe-from height yet, do so now
    // We'll remember it until the machine is reset
    if(probe_from_height == -1) {

        // Fast the first time
        _printf("Determining the probe-from height.\n");
        zprobe->run_probe(steps, true);
        
        // Probe from height = total measured height - height required for the probe not to drag
        probe_from_height = zprobe->zsteps_to_mm(steps) - zprobe->getProbeHeight();
        zprobe->home();

    }

    _printf("Probe-from height = %1.3f\n", probe_from_height);

    // Move to probe_from_height (relative move!)
    zprobe->coordinated_move(NAN, NAN, -probe_from_height, zprobe->getFastFeedrate(), true);

    // Move to probing offset
    // We do these as two seperate steps because the top of a delta's build envelope is domed,
    // and we want to avoid the possibility of asking the effector to move somewhere it can't
    zprobe->coordinated_move(probe_offset_x, probe_offset_y, NAN, zprobe->getFastFeedrate(), false);

//    // Slow down
//    save_acceleration();
//    set_acceleration(probe_acceleration);

    // Prime the probe - this measurement is one of the most important!
//    if(!prime_probe()) {
//        pop_prefix();
//        return false;
//    }

    // Now, (slowly) probe the depth
    if(!run_probe(steps, false, false)  /*!zprobe->run_probe(steps, false)*/) {
//        restore_acceleration();
        pop_prefix();
        return false;
    }
//    restore_acceleration();
    mm_probe_height_to_trigger = zprobe->zsteps_to_mm(steps);

    // Set final bed height
    bed_height = probe_from_height + mm_probe_height_to_trigger + probe_offset_z;

    // Tell the machine about the new height
    // FIXME: Endstops.cpp might have a more direct method for doing this - if so, that should be used instead!
    // -- Construct command
    char cmd[18];       // Should be enough for "M665 Z1000.12345"
    snprintf(cmd, 17, "M665 Z%1.5f", bed_height);
    
    // -- Send command
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    THEKERNEL->conveyor->wait_for_empty_queue();

    _printf("Bed height set to %1.3f\n", bed_height);

    pop_prefix();
    return true;

}


// Do a probe at a specified (X, Y) location, taking probe offset into account
bool ComprehensiveDeltaStrategy::do_probe_at(int &steps, float x, float y, bool skip_smoothing, bool skip_priming, bool ignore_xy) {

    // Move to location, corrected for probe offset (if any)
    if(!ignore_xy) {
        zprobe->coordinated_move(x + probe_offset_x, y + probe_offset_y, NAN, zprobe->getFastFeedrate(), false);
    }

    // Slow down
    save_acceleration();
    set_acceleration(probe_acceleration);

    // Run the probe
    bool success = run_probe(steps, skip_smoothing, skip_priming);

    // Speed up
    restore_acceleration();

    return success;

}


// Run the probe (wrapper for ZProbe->run_probe())
bool ComprehensiveDeltaStrategy::run_probe(int &steps, bool skip_smoothing, bool skip_priming) {

    // Run the number of tests specified in probe_smoothing
    steps = 0;
    int smoothing, result;
    if(skip_smoothing) {
        smoothing = 1;
    } else {
        smoothing = probe_smoothing;
    }

    // Prime the probe (run it a number of times to get it to "settle")
    if(!skip_priming) {
        if(!prime_probe()) {
            pop_prefix();
            return false;
        }
    }

    // Run the probe
    for(int i=0; i < smoothing; i++) {
        if(!zprobe->run_probe(result)) {
            if(i != 0) steps /= i;
            __printf("[RP] run_probe(steps, skip_smoothing=%d, skip_priming=%d) - run_probe() returned false, s=%d. (Was it already triggered?)\n", skip_smoothing, skip_priming, steps);
            restore_acceleration();
            return false;
        }

        // Return probe to original Z
        if(zprobe->getDecelerateOnTrigger()) {
            zprobe->return_probe(zprobe->getStepsAtDecelEnd());
        } else {
            zprobe->return_probe(result);
        }

        // Add to accumulator
        steps += result;

    }

    // Average
    steps /= smoothing;

    // Sanity check
    if(steps < 100) {
        __printf("[DP] do_probe_at(): steps=%d - this is much too small - is probe_height high enough?\n", steps);
        return false;
    } else {
        return true;
    }

}


// The printer has to have its position refreshed when the kinematics change. Otherwise, it will jerk violently the
// next time it moves, because its last milestone (location) was calculated using the previous kinematics.
void ComprehensiveDeltaStrategy::post_adjust_kinematics() {

    float pos[3];
    THEKERNEL->robot->get_axis_position(pos);
    THEKERNEL->robot->reset_axis_position(pos[0], pos[1], pos[2]);

}


// This is the version you want to use if you're fiddling with the endstops. Note that endstop
// offset values are NEGATIVE (steps down).
void ComprehensiveDeltaStrategy::post_adjust_kinematics(float offset[3]) {

    float pos[3];
    THEKERNEL->robot->get_axis_position(pos);
    THEKERNEL->robot->reset_axis_position(pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]);
    geom_dirty = true;

}


// Following are getters/setters for global accelration (not Z-specific)
void ComprehensiveDeltaStrategy::save_acceleration() {
    // We only save it if it's different from what it already is.
    if(saved_acceleration != THEKERNEL->planner->get_acceleration()) {
        saved_acceleration = THEKERNEL->planner->get_acceleration();
    }
}

void ComprehensiveDeltaStrategy::restore_acceleration() {
    set_acceleration(saved_acceleration);
}

void ComprehensiveDeltaStrategy::set_acceleration(float a) {

    char cmd[20];       // Should be enough for "M204 S1234.45678"
    snprintf(cmd, 19, "M204 S%1.5f", a);
    // -- Send command
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    THEKERNEL->conveyor->wait_for_empty_queue();

}


// ----------------------------------------------------------
// Following are getters/setters for delta geometry variables
// ----------------------------------------------------------


// Endstops
bool ComprehensiveDeltaStrategy::set_trim(float x, float y, float z) {

    float t[3] {x, y, z};
    bool ok = PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
//        __printf("[ES] Set trim to: X=%f Y=%f Z=%f\n", x, y, z);
    } else {
        __printf("[ES] Unable to set trim. Are endstops enabled?\n");
    }

    geom_dirty = true;
    return ok;
}

bool ComprehensiveDeltaStrategy::get_trim(float &x, float &y, float &z) {

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


// Arm length
bool ComprehensiveDeltaStrategy::set_arm_length(float arm_length, bool update) {

    geom_dirty = true;

    options['L'] = arm_length;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        return false;
    }

}

bool ComprehensiveDeltaStrategy::get_arm_length(float &arm_length) {

    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        arm_length = options['L'];
        return true;
    } else {
        return false;
    }

}


// Delta radius
bool ComprehensiveDeltaStrategy::set_delta_radius(float delta_radius, bool update) {

    geom_dirty = true;

    options['R'] = delta_radius;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        return false;
    }

}

bool ComprehensiveDeltaStrategy::get_delta_radius(float &delta_radius) {

    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        delta_radius = options['R'];
        return true;
    } else {
        return false;
    }

}


// Tower radius offsets
bool ComprehensiveDeltaStrategy::set_tower_radius_offsets(float x, float y, float z, bool update) {

    geom_dirty = true;

    options['A'] = x;
    options['B'] = y;
    options['C'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        return false;
    }

}

bool ComprehensiveDeltaStrategy::get_tower_radius_offsets(float &x, float &y, float &z) {

    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        x = options['A'];
        y = options['B'];
        z = options['C'];
        return true;
    } else {
        return false;
    }

}


// Tower angle offsets
bool ComprehensiveDeltaStrategy::set_tower_angle_offsets(float x, float y, float z, bool update) {

    geom_dirty = true;

    options['D'] = x;
    options['E'] = y;
    options['F'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        return false;
    }

}

bool ComprehensiveDeltaStrategy::get_tower_angle_offsets(float &x, float &y, float &z) {

    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        x = options['D'];
        y = options['E'];
        z = options['F'];
        return true;
    } else {
        return false;
    }

}


// Arm length offsets
bool ComprehensiveDeltaStrategy::set_tower_arm_offsets(float x, float y, float z, bool update) {

    geom_dirty = true;

    options['T'] = x;
    options['U'] = y;
    options['V'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        return false;
    }

}

bool ComprehensiveDeltaStrategy::get_tower_arm_offsets(float &x, float &y, float &z) {

    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        x = options['T'];
        y = options['U'];
        z = options['V'];
        return true;
    } else {
        return false;
    }

}


// Virtual Shimming (thx Plane3d.cpp)
bool ComprehensiveDeltaStrategy::set_virtual_shimming(float x, float y, float z, bool update) {

//_printf("SVS: {%1.3f, %1.3f, %1.3f}\n", x, y, z);

    geom_dirty = true;

    // Z depths are in millimeters relative to surface, negative=lower
    surface_transform->tri_points[X][Z] = x;
    surface_transform->tri_points[Y][Z] = y;
    surface_transform->tri_points[Z][Z] = z;

    if(x == 0 && y == 0 && z == 0) {

        // This gets its own special case because Vector3.cpp is incapable of handling null vectors.
        // It will literally calculate that the cross product of {0, 0, 0} and {0, 0, 0} is {nan, nan, nan}.
        surface_transform->normal.set(0, 0, 1);
        surface_transform->d = 0;

    } else {

        Vector3 v1, v2, v3;
        v1.set(surface_transform->tri_points[X][X], surface_transform->tri_points[X][Y], surface_transform->tri_points[X][Z]);
        v2.set(surface_transform->tri_points[Y][X], surface_transform->tri_points[Y][Y], surface_transform->tri_points[Y][Z]);
        v3.set(surface_transform->tri_points[Z][X], surface_transform->tri_points[Z][Y], surface_transform->tri_points[Z][Z]);

//        _printf("Vector 1: {%1.3f, %1.3f, %1.3f}\n", v1[X], v1[Y], v1[Z]);
//        _printf("Vector 2: {%1.3f, %1.3f, %1.3f}\n", v2[X], v2[Y], v2[Z]);
//        _printf("Vector 3: {%1.3f, %1.3f, %1.3f}\n", v3[X], v3[Y], v3[Z]);

        Vector3 ab = v1.sub(v2);
        Vector3 ac = v1.sub(v3);
        Vector3 cross_product = ab.cross(ac);

        surface_transform->normal = cross_product.unit();

//        _printf("ab: {%1.3f, %1.3f, %1.3f}\n", ab[X], ab[Y], ab[Z]);
//        _printf("ac: {%1.3f, %1.3f, %1.3f}\n", ac[X], ac[Y], ac[Z]);
//        _printf("cross product: {%1.3f, %1.3f, %1.3f}\n", cross_product[X], cross_product[Y], cross_product[Z]);
//        _printf("normal: {%1.3f, %1.3f, %1.3f}\n", surface_transform->normal[X], surface_transform->normal[Y], surface_transform->normal[Z]);

        Vector3 dv = surface_transform->normal.mul(v1);
//        _printf("dv: {%1.3f, %1.3f, %1.3f}\n", dv[X], dv[Y], dv[Z]);

        surface_transform->d = -dv[0] - dv[1] - dv[2];
//        _printf("d: %1.3f\n", surface_transform->d);

        surface_transform->plane_enabled = true;
        set_adjust_function(true);

    }

//    _printf("normal: {%1.3f, %1.3f, %1.3f}\n", surface_transform->normal[X], surface_transform->normal[Y], surface_transform->normal[Z]);
//    _printf("d: %1.3f\n", surface_transform->d);

    surface_transform->have_normal = true;
    return true;
    
}

bool ComprehensiveDeltaStrategy::get_virtual_shimming(float &x, float &y, float &z) {
    if(surface_transform->plane_enabled) {
        x = surface_transform->tri_points[X][Z];
        y = surface_transform->tri_points[Y][Z];
        z = surface_transform->tri_points[Z][Z];
    } else {
        x = y = z = 0;
    }
    return true;
}


// Getter/setter for ALL kinematics
bool ComprehensiveDeltaStrategy::set_kinematics(KinematicSettings *settings, bool update) {

    if(settings->initialized) {

        set_delta_radius(settings->delta_radius);
        set_arm_length(settings->arm_length);
        set_trim(settings->trim[X], settings->trim[Y], settings->trim[Z]);
        set_tower_radius_offsets(settings->tower_radius[X], settings->tower_radius[Y], settings->tower_radius[Z]);
        set_tower_angle_offsets(settings->tower_angle[X], settings->tower_angle[Y], settings->tower_angle[Z]);
        set_tower_arm_offsets(settings->tower_arm[X], settings->tower_arm[Y], settings->tower_arm[Z]);
        set_virtual_shimming(settings->virtual_shimming[X], settings->virtual_shimming[Y], settings->virtual_shimming[Z]);

        if(update) {
            post_adjust_kinematics();
        }

        return true;

    } else {

        __printf("[SK] Tried to set kinematics to uninitialized settings!\n");
        return false;

    }
}

bool ComprehensiveDeltaStrategy::get_kinematics(KinematicSettings *settings) {

    get_delta_radius(settings->delta_radius);
    get_arm_length(settings->arm_length);
    get_trim(settings->trim[X], settings->trim[Y], settings->trim[Z]);
    get_tower_radius_offsets(settings->tower_radius[X], settings->tower_radius[Y], settings->tower_radius[Z]);
    get_tower_angle_offsets(settings->tower_angle[X], settings->tower_angle[Y], settings->tower_angle[Z]);
    get_tower_arm_offsets(settings->tower_arm[X], settings->tower_arm[Y], settings->tower_arm[Z]);
    get_virtual_shimming(settings->virtual_shimming[X], settings->virtual_shimming[Y], settings->virtual_shimming[Z]);
    settings->initialized = true;
    return true;

}


// Print currently set kinematics
void ComprehensiveDeltaStrategy::print_kinematics() {

    KinematicSettings *settings = new KinematicSettings();
    get_kinematics(settings);
    print_kinematics(settings);

}

void ComprehensiveDeltaStrategy::print_kinematics(KinematicSettings *settings) {

    push_prefix("PK");
    newline();
    _printf("Current kinematic settings:\n");
    _printf("          Arm length: %1.3f\n", settings->arm_length);
    _printf("        Delta radius: %1.3f\n", settings->delta_radius);
    _printf("     Endstop offsets: {%1.3f, %1.3f, %1.3f}\n", settings->trim[X], settings->trim[Y], settings->trim[Z]);
    _printf("Radius offsets (ABC): {%1.3f, %1.3f, %1.3f}\n", settings->tower_radius[X], settings->tower_radius[Y], settings->tower_radius[Z]);
    _printf(" Angle offsets (DEF): {%1.3f, %1.3f, %1.3f}\n", settings->tower_angle[X], settings->tower_angle[Y], settings->tower_angle[Z]);
    _printf("    Virtual shimming: {%1.3f, %1.3f, %1.3f}, vector={%1.3f, %1.3f, %1.3f}, d=%1.3f, %s\n", settings->virtual_shimming[X], settings->virtual_shimming[Y], settings->virtual_shimming[Z], surface_transform->normal[X], surface_transform->normal[Y], surface_transform->normal[Z], surface_transform->d, (surface_transform->plane_enabled && surface_transform->active) ? _STR_ENABLED_ : _STR_DISABLED_);
    _printf("Depth (Z) correction: %s\n", (surface_transform->depth_enabled && surface_transform->active) ? _STR_ENABLED_ : _STR_DISABLED_);
    newline();
    pop_prefix();

}


// Print measured or simulated depths
void ComprehensiveDeltaStrategy::print_depths(cds_depths_t *depths, bool extrapolated) {
    newline();
    for(int i=0; i<DM_GRID_DIMENSION; i++) {
        print_depths_line(depths, i, extrapolated);
    }
}

void ComprehensiveDeltaStrategy::print_depths(float **depths, bool extrapolated) {
    newline();
    for(uint8_t i=0; i<DM_GRID_DIMENSION; i++) {
        print_depths_line(depths, i, extrapolated);
    }
}


void ComprehensiveDeltaStrategy::print_depths_line(float **depths, uint8_t line, bool extrapolated) {

    // FIXME: This could be made more memory efficient & elegant (only copy those for the line we care about)

    cds_depths_t *_depths;
    if((_depths = (cds_depths_t *)AHB0.alloc(sizeof(cds_depths_t) * DM_GRID_ELEMENTS)) == nullptr) {
        __printf("[PD] ERROR: Couldn't allocate RAM.\n");
        return;
    }
    
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        _depths[i].abs = 0;
        _depths[i].rel = depths[i][Z];
    }

    print_depths_line(_depths, line, extrapolated);

    AHB0.dealloc(_depths);

}

void ComprehensiveDeltaStrategy::print_depths_line(cds_depths_t *depths, uint8_t line, bool extrapolated) {

//    float rel_depths[DM_GRID_ELEMENTS];
//    float *rel_depths = (float *)AHB0.alloc(DM_GRID_ELEMENTS * sizeof(float));
//    float best = 999, worst = 0;
//    float mu, sigma, min, max;

    // Print header
    __printf("[PD] ");

    int i;

    // Print depths
    int col = 0;
    for(i=0; i<DM_GRID_DIMENSION; i++) {
    
        int idx = (DM_GRID_DIMENSION * line) + i;

        // Print entry (or a blank space, if the test point is turned off)
        switch(active_point[idx]) {
            case TP_CENTER:
            case TP_ACTIVE:
                __printf(" %6.3f ", depths[idx].rel);
                break;
            case TP_ACTIVE_NEIGHBOR:
                if(extrapolated) {
                    __printf("[%6.3f]", depths[idx].rel);
                } else {
                    __printf("[  --  ]");
                }
                break;
            case TP_INACTIVE:
                 __printf("        ");
                 break;
        }

        // Space or new line?
        if(col++ < DM_GRID_DIMENSION - 1) {
            __printf("   ");
        } else if(i < DM_GRID_ELEMENTS - 1) {
//            col = 0;
            __printf("\n[PD]\n");
        }
        
        flush();

    }

//    AHB0.dealloc(rel_depths);

}


// Statistics for probed depths
void ComprehensiveDeltaStrategy::print_depths_statistics(cds_depths_t *depths) {

    float *rel_depths = (float *)AHB0.alloc(DM_GRID_ELEMENTS * sizeof(float));
    float best = 999, worst = 0;
    float mu, sigma, min, max;
    int i;

    // Calc statistics
    for(i=0; i<DM_GRID_ELEMENTS; i++) {

        // Statistics calc requires a one-dimensional array
        rel_depths[i] = depths[i].rel;

        // Do some statistics (sign doesn't matter, only magnitude)
        if(fabs(depths[i].rel) < fabs(best)) {
            best = fabs(depths[i].rel);
        }

        if(fabs(depths[i].rel) > fabs(worst)) {
            worst = fabs(depths[i].rel);
        }

    }

    AHB0.dealloc(rel_depths);

    // The difference between "best/worst" and "min/max" is that best and worst are indifferent to sign.
    calc_statistics(rel_depths, DM_GRID_ELEMENTS, mu, sigma, min, max);
    float energy = calc_energy(depths);
    __printf("[PD] Best=%1.3f, worst=%1.3f, min=%1.3f, max=%1.3f, mu=%1.3f, sigma=%1.3f, energy=%1.3f\n", best, worst, min, max, mu, sigma, energy);
    newline();

}


// Distance between two points in 2-space
float ComprehensiveDeltaStrategy::distance2D(float first[2], float second[2]) {
    return sqrt(pow(second[X] - first[X], 2) + pow(second[Y] - first[Y], 2));
}


// Distance between two points in 3-space
float ComprehensiveDeltaStrategy::distance3D(float first[3], float second[3]) {
    return sqrt(
        pow(second[X] - first[X], 2) +
        pow(second[Y] - first[Y], 2) +
        pow(second[Z] - first[Z], 2)
        );
}


// Rotate a point around another point in 2-space.
// Adapted from http://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
void ComprehensiveDeltaStrategy::rotate2D(float (&point)[2], float reference[2], float angle) {

    float s = sin(angle * 3.141595 / 180.0);
    float c = cos(angle * 3.141595 / 180.0);
    
    point[X] -= reference[X];
    point[Y] -= reference[Y];

    float xNew = point[X] * c - point[Y] * s;
    float yNew = point[X] * s + point[Y] * c;

    point[X] = xNew + reference[X];
    point[Y] = yNew + reference[Y];

}


// Blink the idle loop LED.
void ComprehensiveDeltaStrategy::blink_LED(unsigned char which) {

    if(THEKERNEL->use_leds) {
        switch(which) {
            case 1:
            case 2:
                if(GETBIT(LED_state, which)) {
                    CLRBIT(LED_state, which);
                    leds[which] = 0;
                } else {
                    SETBIT(LED_state, which);
                    leds[which] = 1;
                }
                break;
            default:
                __printf("[BL] ERROR: Can only blink LEDs 0 and 1!\n");
                break;
        }
    }

}

// Zero out depth_map.
void ComprehensiveDeltaStrategy::zero_depth_maps() {

    for(int i=0; i < DM_GRID_ELEMENTS; i++) {
        depth_map[i].abs = 0;
        depth_map[i].rel = 0;
    }

}


// Copy a depth map to another depth map
void ComprehensiveDeltaStrategy::copy_depth_map(cds_depths_t source[], cds_depths_t dest[]) {

    for(int i=0; i < DM_GRID_ELEMENTS; i++) {
        dest[i].abs = source[i].abs;
        dest[i].rel = source[i].rel;
    }

}


// Turn off all calibration types
void ComprehensiveDeltaStrategy::clear_calibration_types() {

    caltype.endstop.active = false;
    caltype.delta_radius.active = false;
    caltype.arm_length.active = false;
    caltype.tower_angle.active = false;
    caltype.virtual_shimming.active = false;

}


// Display active/inactive calibration types.
// The args are either-or - they shouldn't both be true.
void ComprehensiveDeltaStrategy::display_calibration_types(bool active, bool inactive) {

    char ES[] = "Endstops (O)";
    char DR[] = "Delta Radius (P)";
    char AL[] = "Arm Length (Q)";
    char TAO[] = "Tower Angle Offset (R)";
    char VS[] = "Virtual Shimming (S)";
    char format[] = "[%s, mul=%1.2f] ";
    int nShown = 0;

    // Display active caltypes
    if(active) {

        if(caltype.endstop.active) {
            __printf(format, ES, caltype.endstop.annealing_temp_mul);
            nShown++;
        }

        if(caltype.delta_radius.active) {
            __printf(format, DR, caltype.delta_radius.annealing_temp_mul);
            nShown++;
        }

        if(caltype.arm_length.active) {
            __printf(format, AL, caltype.arm_length.annealing_temp_mul);
            nShown++;
        }

        if(caltype.tower_angle.active) {
            __printf(format, TAO, caltype.tower_angle.annealing_temp_mul);
            nShown++;
        }
        
        if(caltype.virtual_shimming.active) {
            __printf(format, VS, caltype.virtual_shimming.annealing_temp_mul);
            nShown++;
        }

    } // active    

    // Display inactive caltypes
    if(inactive) {

        if(!caltype.endstop.active) {
            __printf(format, ES, caltype.endstop.annealing_temp_mul);
            nShown++;
        }

        if(!caltype.delta_radius.active) {
            __printf(format, DR, caltype.delta_radius.annealing_temp_mul);
            nShown++;
        }

        if(!caltype.arm_length.active) {
            __printf(format, AL, caltype.arm_length.annealing_temp_mul);
            nShown++;
        }

        if(!caltype.tower_angle.active) {
            __printf(format, TAO, caltype.tower_angle.annealing_temp_mul);
            nShown++;
        }

        if(!caltype.virtual_shimming.active) {
            __printf(format, VS, caltype.virtual_shimming.annealing_temp_mul);
            nShown++;
        }

    } // inactive

    // Print a nice placeholder if no caltypes were active/inactive
    if(nShown == 0) {
        __printf("(none)");
    }

    newline();

}


// Calculate mean (mu), standard deviation (sigma), min, and max values for an array of arbitrary length
void ComprehensiveDeltaStrategy::calc_statistics(float values[], int n_values, float &mu, float &sigma, float &min, float &max) {

    // Init
    int stats;
    float dev;
    min =  999;
    max = -999;

    // Mu, min, and max
    mu = 0;
    for(stats = 0; stats < n_values; stats++) {
        mu += values[stats];
        if(values[stats] > max) { max = values[stats]; }
        if(values[stats] < min) { min = values[stats]; }
    }
    mu /= n_values;

    // Sigma
    dev = 0;
    for(stats=0; stats < n_values; stats++) {
        dev += powf((float)values[stats] - mu, 2);
    }
    sigma = sqrtf(dev/n_values);

}


// Calculate the "energy" of an array of depths
float ComprehensiveDeltaStrategy::calc_energy(cds_depths_t *points) {

//    float cartesian[DM_GRID_ELEMENTS][3];
//    float **cartesian;
//    if((cartesian = AHB0_alloc2Df(DM_GRID_ELEMENTS, 3)) == nullptr) {
//        __printf("[CE] Couldn't allocate temp Cartesian array.\n");
//        return 999;
//    }

    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        temp_cartesian[i][X] = test_point[i][X];
        temp_cartesian[i][Y] = test_point[i][Y];
        temp_cartesian[i][Z] = points[i].rel;
//        _printf("cartesian[%d] = %1.1f, %1.1f, %1.1f\n", i, test_point[i][X], test_point[i][Y], points[i].rel);
        flush();
    }

    float energy = calc_energy(temp_cartesian);

//    AHB0_dealloc2Df(cartesian, DM_GRID_ELEMENTS);

    return energy;

}

float ComprehensiveDeltaStrategy::calc_energy(float **cartesian) {

    float mu = 0;
    int i = 0;

    for(int stats = 0; stats < DM_GRID_ELEMENTS; stats++) {
        if(active_point[stats] == TP_ACTIVE) {
            mu += fabs(cartesian[stats][Z]);
            i++;
        }
    }

    return mu / i;

}


// Calculate the midpoint of a 2-D line.
// first[] and second[] are floats. Resulting midpoint stored in dest[].
void ComprehensiveDeltaStrategy::midpoint(float first[2], float second[2], float (&dest)[2]) {

    dest[0] = (first[0] + second[0]) / 2;
    dest[1] = (first[1] + second[1]) / 2;

}


// Make sure n is between lower and upper
float ComprehensiveDeltaStrategy::clamp(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}


// Print some spaces
void ComprehensiveDeltaStrategy::str_pad_left(unsigned char spaces) {
    for(unsigned char i = 0; i < spaces; i++) {
        __printf(" ");
    }
}


// Print a banner indicating what we're working on, and what a terrible idea it would be to touch the printer
// in any way (except for the reset button)
void ComprehensiveDeltaStrategy::print_task_with_warning(const std::string& str) {
    char str_[str.length() + 1];	// Plus 1 to accommodate a \0... str.length() doesn't account for that!
    std::strcpy(str_, str.c_str());
    newline();
    _printf("%s in progress. Press Reset to abort.\n", str_);
    _printf("/!\\ PROBE CRASH DANGER /!\\ Don't press buttons, send commands, or access the SD card.\n \n");
}


// Allow the kernel to flush the serial buffer, and perform whatever other maintenance tasks are needed
// Note: It would be a good idea to avoid doing anything to the kernel that would hang it ON_IDLE.
void ComprehensiveDeltaStrategy::flush() {
    THEKERNEL->call_event(ON_IDLE);
}

void ComprehensiveDeltaStrategy::newline() {
    THEKERNEL->streams->printf(" \n");
}


// Fetch the bed temperature
bool ComprehensiveDeltaStrategy::get_bed_temp(float &temp, float &target) {

    void *returned_data;
    string type = "bed";
    bool ok = PublicData::get_value( temperature_control_checksum, get_checksum(type), current_temperature_checksum, &returned_data );
    if(!ok) {
        _printf("ERROR: Couldn't query bed temperature!\n");
        return false;
    }
    struct pad_temperature _temp = *static_cast<struct pad_temperature *>(returned_data);
    temp = _temp.current_temperature;
    target = _temp.target_temperature;

    return true;

}


// Method Prefixes
// Rather than _printf("[xx] thing"), where "[xx] " is repeated in dozens to hundreds of _printf() statements,
// we automate the prefix, the idea being to save RAM.
void ComprehensiveDeltaStrategy::print_method_prefix() {
    if(method_prefix[method_prefix_idx][0] != 0) {
        THEKERNEL->streams->printf("[%s] ", method_prefix[method_prefix_idx]);
    }
}

void ComprehensiveDeltaStrategy::push_prefix(const std::string& mp) {
    if(method_prefix_idx + 1 < MP_MAX_PREFIXES) {
        strncpy(method_prefix[++method_prefix_idx], mp.c_str(), 3);
    } else {
        THEKERNEL->streams->printf("Prefix: Max prefixes exceeded (%d)\n", method_prefix_idx);
    }
}

void ComprehensiveDeltaStrategy::pop_prefix() {
    if(method_prefix_idx > 0) {
        method_prefix_idx--;
    } else {
        THEKERNEL->streams->printf("Prefix: Tried to pop one too many times\n");
    }
}


