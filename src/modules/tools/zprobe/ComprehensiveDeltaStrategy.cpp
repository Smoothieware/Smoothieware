/*

    Comprehensive Delta Strategy
    
    G-codes:	G29	Probe Calibration
                        
    
                G31	Heuristic Calibration
                G32	Iterative Calibration


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

#include <time.h>
#include <tuple>
#include <algorithm>

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
#define _printf THEKERNEL->streams->printf


// This serves in place of a constructor; it will be called whenever the config is reloaded
// (which you can do over a serial console, by the way)
bool ComprehensiveDeltaStrategy::handleConfig() {

    // Set probe_from_height to a value that find_bed_center_height() will know means it needs to be initialized
    probe_from_height = -1;

    // Set the dirty flag, so we know we have to calibrate the endstops and delta radius
    geom_dirty = true;
    
    // Turn off Z compensation (for now)
    z_compensation_enabled = false;

    // Zero out depth_map and test_depth_map
    zero_depth_maps();

    // Turn off all calibration types
    clear_calibration_types();

    // TODO: Read this from config_override via M-code
    surface_shape = PSS_CIRCLE;

    // Initialize test points
    init_test_points();
    
    // Initialize the best probe calibration stats (we'll use sigma==-1 to check whether initialized)
    best_probe_calibration.sigma = -1;
    best_probe_calibration.range = -1;
    best_probe_calibration.accel = -1;
    best_probe_calibration.debounce_count = -1;
    best_probe_calibration.decelerate = false;
    best_probe_calibration.eccentricity = true ;
    best_probe_calibration.smoothing = -1;
    best_probe_calibration.fast = -1;
    best_probe_calibration.slow = -1;
    
    // Probe radius
    float r = THEKERNEL->config->value(leveling_strategy_checksum, comprehensive_delta_strategy_checksum, probe_radius_checksum)->by_default(-1)->as_number();
    if(r == -1) {
        // Deprecated config syntax
        r =  THEKERNEL->config->value(zprobe_checksum, probe_radius_checksum)->by_default(100.0F)->as_number();
    }
    this->probe_radius = r;

    // Initialize bilinear interpolation array scaler (requires probe_radius)
    bili.cartesian_to_array_scaler = (DM_GRID_DIMENSION - 1) / (probe_radius * 2);

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
    // - Jog the probe up enough to remove the dot, and do so
    // - Jog the probe back down again until it triggers (use tiny moves to get it as accurate as possible)
    // - Record the position in config as probe_offset_x/y/z
    this->probe_offset_x = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_x_checksum)->by_default(0)->as_number();
    this->probe_offset_y = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_y_checksum)->by_default(0)->as_number();
    this->probe_offset_z = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_z_checksum)->by_default(0)->as_number();

    // TODO: Load surface transform from config-override (some M-code, I suppose)

    init_test_points();



/*
    // Towers are 60 degrees off centerline.
    // So, the quadrants look like this:
    // Q2: -xDeg, +yDeg   Q1: +xDeg, +yDeg
    // Q3: -xDeg, -yDeg   Q4: +xDeg, -yDeg
    float xDeg = 0.866025f;
    float yDeg = 0.5;

    // Points at towers (this is simple quadrant stuff)
    test_point[TP_X][X] = -xDeg * probe_radius;
    test_point[TP_X][Y] = -yDeg * probe_radius;
    test_point[TP_Y][X] =  xDeg * probe_radius;
    test_point[TP_Y][Y] = -yDeg * probe_radius;
    test_point[TP_Z][X] =                    0;
    test_point[TP_Z][Y] =         probe_radius;

    // Points opposite towers
    // Merely a sign-flipped version of above, so the points are mirrored about the origin
    test_point[TP_OPP_X][X] =  xDeg * probe_radius;
    test_point[TP_OPP_X][Y] =  yDeg * probe_radius;
    test_point[TP_OPP_Y][X] = -xDeg * probe_radius;
    test_point[TP_OPP_Y][Y] =  yDeg * probe_radius;
    test_point[TP_OPP_Z][X] =                    0;
    test_point[TP_OPP_Z][Y] =        -probe_radius;

    // Midpoints between towers
    midpoint(test_point[TP_X], test_point[TP_Y], test_point[TP_MID_XY]);
    midpoint(test_point[TP_Y], test_point[TP_Z], test_point[TP_MID_YZ]);
    midpoint(test_point[TP_Z], test_point[TP_X], test_point[TP_MID_ZX]);

    // Opposite midpoints between towers
    // These happen to be halfway between {0, 0} and the points opposite the X/Y/Z towers
    test_point[TP_OPP_MID_XY][X] = test_point[TP_MID_XY][X];
    test_point[TP_OPP_MID_XY][Y] = -test_point[TP_MID_XY][Y];
    test_point[TP_OPP_MID_ZX][X] = test_point[TP_OPP_X][X] / 2;
    test_point[TP_OPP_MID_ZX][Y] = -test_point[TP_OPP_X][Y] / 2;
    test_point[TP_OPP_MID_YZ][X] = test_point[TP_OPP_Y][X] / 2;
    test_point[TP_OPP_MID_YZ][Y] = -test_point[TP_OPP_Y][Y] / 2;
*/

    return true;

}


// Process incoming G- and M-codes
bool ComprehensiveDeltaStrategy::handleGcode(Gcode *gcode) {

    if( gcode->has_g) {
        // G code processing
        if(gcode->g == 29) { // Test the Z-probe for repeatability

            measure_probe_repeatability(gcode);
            return true;

        }

        if(gcode->g == 31) { // Depth-map the bed and display the results

            if(gcode->has_letter('A')) {

                int x, y, dm_pos;
/*
                // Testing
                determine_active_points();
                _printf("Before propagation:\n");
                for(y=0; y<DM_GRID_DIMENSION; y++) {
                    for(x=0; x<DM_GRID_DIMENSION; x++) {
                        surface_transform[(y * DM_GRID_DIMENSION) + x] = (-x + y);
                        _printf("%1.3f   ", surface_transform[(y * DM_GRID_DIMENSION) + x]);
                    }
                    _printf("\n");
                }

*/
                // Build depth map

                float cartesian[DM_GRID_ELEMENTS][3];
                depth_map_print_surface(cartesian, RESULTS_FORMATTED);

                // Copy depth map to surface transform, which contains depths only
                for(int i=0; i<DM_GRID_ELEMENTS; i++) {
                    surface_transform[i] = cartesian[i][Z];
                }


                // Propagate values outward from circle to edge, in case they go outside probe_radius
                if(surface_shape == PSS_CIRCLE) {
                    for(y=0; y<DM_GRID_DIMENSION; y++) {
                        for(x=0; x <= (DM_GRID_DIMENSION-1) / 2; x++) {

                            // Propagate right
                            dm_pos = (y * DM_GRID_DIMENSION) + ((DM_GRID_DIMENSION - 1) / 2) + x;
                            if(!active_point[dm_pos]) {
                                surface_transform[dm_pos] = surface_transform[dm_pos - 1];
                            }

                            // Propagate left
                            dm_pos = (y * DM_GRID_DIMENSION) + ((DM_GRID_DIMENSION - 1) / 2) - x;
                            if(!active_point[dm_pos]) {
                                surface_transform[dm_pos] = surface_transform[dm_pos + 1];
                            }

                        }
                    }
                }
/*
                // Print propagated depth map
                _printf(" \n");
                _printf("After propagation:\n");
                for(y=0; y<DM_GRID_DIMENSION; y++) {
                    for(x=0; x<DM_GRID_DIMENSION; x++) {
                        _printf("%1.3f   ", surface_transform[(y * DM_GRID_DIMENSION) + x]);
                    }
                    _printf("\n");
                }
*/

/*
                // Testing
                // Diagonal, top left to bottom right
                _printf(" \n");
                float _x, _y;
                for(_x=-probe_radius; _x<=probe_radius; _x+=10) {
                    _y = -_x;
                    _printf("{%1.2f, %1.2f}: ", _x, _y);
                    _printf("%1.3f\n", get_adjust_z(_x, _y));
                }

                // From left to right across X axis
                _printf(" \n");
                for(_x=-probe_radius; _x<=probe_radius; _x+=10) {
                    _y = 0;
                    _printf("{%1.2f, %1.2f}: ", _x, _y);
                    _printf("%1.3f\n", get_adjust_z(_x, _y));
                }
*/

                // Enable depth correction
                set_adjust_function(true);

            } else if(gcode->has_letter('B')) {

                // Turn depth correction on or off
                set_adjust_function(!z_compensation_enabled);

            } else if(gcode->has_letter('Z')) {

                // We are only here to map the surface - no calibration
                _printf("[DM] Current kinematics:\n");
                print_kinematics();
                _printf(" \n");
                float dummy[DM_GRID_ELEMENTS][3];
                depth_map_print_surface(dummy, RESULTS_FORMATTED);

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

                // Keep settings?
                if(gcode->has_letter('K')) {
                    keep_settings = true;
                }

                // Simulate-only
                if(gcode->has_letter('O')) {
                    simulate_only = true;
                }

                // Endstops
                if(gcode->has_letter('P')) {
                    caltype.endstop.active = true;
                    caltype.endstop.annealing_temp_mul = gcode->get_value('P');
                }
                
                // Delta radius, including individual tower offsets
                if(gcode->has_letter('Q')) {
                    caltype.delta_radius.active = true;
                    caltype.delta_radius.annealing_temp_mul = gcode->get_value('Q');
                }

                // Arm length, including individual arm length offsets
                if(gcode->has_letter('R')) {
                    caltype.arm_length.active = true;
                    caltype.arm_length.annealing_temp_mul = gcode->get_value('R');
                }

                // Tower angle offsets
                if(gcode->has_letter('S')) {
                    caltype.tower_angle.active = true;
                    caltype.tower_angle.annealing_temp_mul = gcode->get_value('S');
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

                if(gcode->get_num_args() > 0) {

                    // Make sure at least one caltype is turned on
                    if(
                        !caltype.endstop.active &&
                        !caltype.delta_radius.active &&
                        !caltype.arm_length.active &&
                        !caltype.tower_angle.active
                    ){
                        _printf("[HC] No calibration types selected - activating endstops & delta radius.\n");
                        caltype.endstop.active = true;
                        caltype.delta_radius.active = true;
                    }

                    heuristic_calibration(annealing_tries, max_temp, binsearch_width, simulate_only, keep_settings, zero_all_offsets, overrun_divisor);
                
                } else {
                
                    flush();
                    char _on[] = "on";
                    char _off[] = "off";
                    _printf("[HC] G31 usage: (* = you can supply an annealing multiplier)\n");
                    _printf("[HC] A: Set up depth map for auto leveling\n");
                    _printf("[HC] B: Enable or disable auto leveling (currently %s)\n", z_compensation_enabled ? _on : _off);
                    _printf("[HC] Z: Probe and display depth map - no calibration\n");
                    _printf("[HC] K: Keep last settings\n");
                    _printf("[HC] O: Simulate only (don't probe)\n");
                    _printf("[HC] P: Endstops *\n");
                    _printf("[HC] Q: Delta radius *\n");
                    _printf("[HC] R: Arm length *\n");
                    _printf("[HC] S: Tower angle offsets *\n");
                    _printf("[HC] T: Annealing: Iterations (50)\n");
                    _printf("[HC] U: Annealing: Max t_emp (0.35)\n");	// Repetier Host eats *any* line containing "temp"
                    _printf("[HC] V: Annealing: Binary search width (0.1)\n");
                    _printf("[HC] W: Annealing: Overrun divisor (2)\n");
                    _printf("[HC] Y: Zero all individual radius, angle, and arm length offsets\n");
                    flush();
                }
            
            } // !gcode->has_letter('M')

            return true;

        }

        if(gcode->g == 32) { // Auto calibration for delta, Z bed mapping for cartesian
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            // Comprehensive strategy:
            // - Two tolerances
            //   - Permitted depth variation between all points, e.g. 50 microns
            //   - Depth variation worsening permitted during a strategy test, i.e., the test fails if it perturbs the
            //     depths of points not on its tower-opposite line by more than this amount, e.g. 30 microns(?)
            //   - This means we have to keep track of last depth map to compare it with current
            // - Level endstops
            // - Calibrate printer radius
            // - Depth-map the bed
            //	 - Good enough? Done
            //   - Not? Use three strategies to try to improve it
            //     - Test a line from each tower to its opposite (probe multiple points)
            //     - Try modifying tower's arm length
            //     - Try modifying tower's delta radius
            //     - Try modifying tower's angle
            //     - Endstops and printer radius to be recalibrated after each strategy, every time an adjustment is tried
            //     - Scores are tabulated
            //	     - Strategy that produces the most gains for its tower has the best (LOWEST!) score
            //       - We also track how bad the change is for points NOT on the tower-opposite line
            //       - A strategy that has the best (LOWEST) score, but gets the other test points out of whack beyond a
            //         specified tolerance, will result in the tower being left alone (no adjustment at all)
            //       - At the end, if a height map reveals that the printer is still outside target tolerance, it can
            //         suggest that the user run the calibration again with an easier (larger) tolerance

            iterative_calibration();

            return true;
        }


    } else if(gcode->has_m) {

        // If the geometry is modified externally, we set the dirty flag (but not for Z - that requires no recalibration)
        if(gcode->m == 665) {
            char letters[] = "ABCDEFTUVLR";
            for(unsigned int i=0; i<strlen(letters); i++) {
                if(gcode->has_letter(letters[i])) {
                    geom_dirty = true;
                }
            }
        }
        
        
    }

    return false;

}




// Main heuristic calibration routine
// This expects caltype.___.active to be set true/false beforehand
bool ComprehensiveDeltaStrategy::heuristic_calibration(int annealing_tries, float max_temp, float binsearch_width, bool simulate_only, bool keep_settings, bool zero_all_offsets, float overrun_divisor) {


/*
        Simulated annealing notes
        
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


    // Banner
    char task[] = "HC";
    char str[] = "Heuristic calibration";
    print_task_with_warning(task, str);

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
    
    // Ensure parallel annealing temp multipliers aren't crazy
    // I like 5, 10, 3, 1.5
    caltype.endstop.annealing_temp_mul = clamp(caltype.endstop.annealing_temp_mul, 0, 50);
    caltype.delta_radius.annealing_temp_mul = clamp(caltype.delta_radius.annealing_temp_mul, 0, 50);
    caltype.arm_length.annealing_temp_mul = clamp(caltype.arm_length.annealing_temp_mul, 0, 50);
    caltype.tower_angle.annealing_temp_mul = clamp(caltype.tower_angle.annealing_temp_mul, 0, 50);

    // Zero offsets, if requested
    if(zero_all_offsets) {
        set_trim(0, 0, 0);
        set_tower_radius_offsets(0, 0, 0);
        set_tower_angle_offsets(0, 0, 0);
        set_tower_arm_offsets(0, 0, 0);
        get_kinematics(base_set);
        get_kinematics(cur_set);
    }

    // Is it live, or is it Memorex?
    char _sim[] = "Simulation (W)";
    char _probe[] = "Probe";
    _printf("[HC]             Data source: %s\n", simulate_only ? _sim : _probe);

    // Display values used, along with the G-codes used to set them
    _printf("[HC]            Active tests: ");
    display_calibration_types(true, false);
    _printf("[HC]          Inactive tests: ");
    display_calibration_types(false, true);

    char _true[] = "true";
    char _false[] = "false";
    _printf("[HC]  Keep last settings (K): %s\n", keep_settings ? _true : _false);
    _printf("[HC]     Annealing tries (T): %d\n", annealing_tries);
    _printf("[HC]            Max temp (U): %1.3f\n", max_temp);
    _printf("[HC] Binary search width (V): %1.3f\n", binsearch_width);
    _printf("[HC]     Overrun divisor (W): %1.3f\n", overrun_divisor);
    _printf("[HC]    Zero all offsets (Y): %s\n", zero_all_offsets ? _true : _false);
    _printf(" \n");

    // Make sure the depth maps are blank
    zero_depth_maps();



    // *******************************************************************
    // * Run a simulated annealing to get the printer config most likely *
    // * to produce what the real printer is doing                       *
    // *******************************************************************

    srand(time(NULL));
    
    // We need to save the kinematic settings for later
    if(!simulate_only || !base_set.initialized) {
        _printf("[HC] Baseline kinematics updated.\n");
        get_kinematics(base_set);
    }

    // Make sure cur_set is initialized
    if(!cur_set.initialized) {
        get_kinematics(cur_set);
    }

    // If we aren't keeping the kinematic settings, copy the base settings into the current settings
    // If not simulating, we need to stay with the last kinematics because they may have changed
    // (whereas, in simulation, they never change)
    if(keep_settings || !simulate_only) {
        _printf("[HC] Keeping existing kinematics.\n");
        get_kinematics(cur_set);
    } else {
        _printf("[HC] Restoring baseline kinematics.\n");
        base_set.copy_to(cur_set);
        set_kinematics(cur_set);
    }

    // Tests (min, max, value|TEST_INIT_MIDRANGE))
    // Main tests:
    TestConfig test_endstop[3] { {-5, 0}, {-5, 0}, {-5, 0} };
    TestConfig test_delta_radius(cur_set.delta_radius - 5, cur_set.delta_radius + 5);
    TestConfig test_arm_length(cur_set.arm_length - 5, cur_set.arm_length + 5);
    TestConfig test_angle_offset[3] { {-3, 3}, {-3, 3}, {-3, 3} };

    // Offsets that tie into the main tests:
    TestConfig test_delta_radius_offset[3] { {-3, 3}, {-3, 3}, {-3, 3} };
    TestConfig test_arm_length_offset[3] { {-3, 3}, {-3, 3}, {-3, 3} };

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
    float cur_cartesian[DM_GRID_ELEMENTS][3];
    int j, k;
    int try_mod_10;				// Will be set to annealing_try % 10
    float lowest;				// For finding the lowest absolute value of three variables

    // Keep track of energy so that we can bail if the annealing stalls
    #define LAST_ENERGY_N 3
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

                _printf("[HC] Perturbing simulated printer parameters.\n");

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
                }

                if(caltype.arm_length.active) {
                    set_arm_length(269.75);
                }

                if(caltype.tower_angle.active) {
                    set_tower_angle_offsets(1, 0, -1.5);
                }
                
                // Save the perturbed kinematics
                get_kinematics(cur_set);

                // Trigger regen of carriage positions
                need_to_simulate_IK = true;

                _printf("[HC] After hosing the variables, the settings are now:\n");
                print_kinematics();

            } // !keep_settings

        } else { // !simulate_only

            // Doing it for real: Get values from probe
            // depth_map[] will contain measured depths relative to center

            if(!keep_settings) {

                _printf("[HC] Depth-mapping the print surface...\n");
                print_kinematics();
                depth_map_print_surface(cur_cartesian, RESULTS_FORMATTED);

            } else {

                _printf("[HC] Keeping old depth map.\n");

            }

        } // simulate_only


        // ***************************************************************
        // * Figure out the actuator positions,                          *
        // * given a printer that ~perfectly~ matches the current config *
        // ***************************************************************

        // Generated test positions => cur_cartesian, generated axis positions => test_axis[] (class member)
        if(need_to_simulate_IK || !simulate_only) {
            _printf("[HC] Generating carriage positions for a printer with this configuration.\n");
            simulate_IK(cur_cartesian, cur_set.trim);
            if(restore_from_temp_set) {
                temp_set.copy_to(cur_set);
                set_kinematics(cur_set);
            }
            need_to_simulate_IK = false;
        } else {
            _printf("[HC] Keeping previously-generated carriage positions.\n");
        }

        _printf(" \n[HC] Starting test configuration: Arm Length=%1.3f, Delta Radius=%1.3f\n \n", cur_set.arm_length, cur_set.delta_radius);


        // ***********************
        // * Simulated Annealing *
        // ***********************

        // Get energy of initial state
        float energy = calc_energy(cur_cartesian);
        _printf(" \n");
        _printf("[HC] ***** Simulated annealing pass %d of %d in progress *****\n", outer_try + 1, outer_tries);
        _printf("[HC] Existing calibration has energy %1.3f\n \n", energy);
        _printf("[HC] Reticulating splines...\n");

        for(annealing_try=0; annealing_try<annealing_tries; annealing_try++) {

            // Set the annealing temperature
            tempFraction = (float)annealing_try / (float)annealing_tries;
            temp = max_temp - (tempFraction * max_temp);
            if(temp < 0.01) {
                temp = 0.01;
            }
            
            try_mod_10 = annealing_try % 10;

            // ****************
            // * Delta Radius *
            // ****************

            if(caltype.delta_radius.active) {

                // Find the best tower (delta) radius offsets
                for(k=0; k<3; k++) {
                    best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_tower_radius_offsets, cur_set.tower_radius, k, test_delta_radius_offset[k].range_min, test_delta_radius_offset[k].range_max, binsearch_width, cur_cartesian, target);
                    move_randomly_towards(cur_set.tower_radius[k], best_value, temp * caltype.delta_radius.annealing_temp_mul, target, overrun_divisor);
                }

                // Find the tower radius with the lowest absolute value
                lowest = 999;
                for(k=0; k<3; k++) {
                    if(fabs(cur_set.tower_radius[k]) < lowest) {
                        lowest = cur_set.tower_radius[k];
                    }
                }

                // Steal that value from the individual radius settings and give it to the global radius setting
                for(k=0; k<3; k++) {
                    cur_set.tower_radius[k] -= lowest;
                }
                cur_set.delta_radius += lowest;

                // Tell the robot what the new delta radius & offsets are
                set_delta_radius(cur_set.delta_radius, false);
                set_tower_radius_offsets(cur_set.tower_radius[X], cur_set.tower_radius[Y], cur_set.tower_radius[Z], false);

            } // caltype.delta_radius.active


            // **************
            // * Arm Length *
            // **************
            
            if(caltype.arm_length.active) {

              	best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_arm_length, test_arm_length.range_min, test_arm_length.range_max, binsearch_width, cur_cartesian, target);
              	move_randomly_towards(cur_set.arm_length, best_value, temp * caltype.arm_length.annealing_temp_mul, target, overrun_divisor);
              	set_arm_length(cur_set.arm_length, false);
/*
                // Find the best arm length offsets
//                for(k=0; k<3; k++) {
k = annealing_try % 3;
                    best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_tower_arm_offsets, cur_set.tower_arm, k, test_arm_length_offset[k].range_min, test_arm_length_offset[k].range_max, binsearch_width, cur_cartesian, target);
                    move_randomly_towards(cur_set.tower_arm[k], best_value, temp * caltype.arm_length.annealing_temp_mul, target, overrun_divisor);
//                }

                // Find the arm length offset with the lowest absolute value
                lowest = 999;
                for(k=0; k<3; k++) {
                    if(fabs(cur_set.tower_arm[k]) < lowest) {
                        lowest = cur_set.tower_arm[k];
                    }
                }

                // Steal that value from the individual arm length settings and give it to the global arm length setting
                for(k=0; k<3; k++) {
                    cur_set.tower_arm[k] -= lowest;
                }
                cur_set.arm_length += lowest;

                // Tell the robot what the new arm length & offsets are            
                set_arm_length(cur_set.arm_length, false);
                set_tower_arm_offsets(cur_set.tower_arm[X], cur_set.tower_arm[Y], cur_set.tower_arm[Z], false);
/**/
            }


            // ************
            // * Endstops *
            // ************

            if(caltype.endstop.active) {
                for(k=0; k<3; k++) {
                    best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_test_trim, cur_set.trim, k, test_endstop[k].range_min, test_endstop[k].range_max, binsearch_width, cur_cartesian, target);
                    move_randomly_towards(cur_set.trim[k], best_value, temp * caltype.endstop.annealing_temp_mul, target, overrun_divisor);
                }

                // Set trim
                set_trim(cur_set.trim[X], cur_set.trim[Y], cur_set.trim[Z]);

            } // caltype.endstop.active


            // ****************
            // * Tower angles *
            // ****************

            if(caltype.tower_angle.active) {

//                k = annealing_try % 3;
                for(k=0; k<3; k++) {
                    best_value = find_optimal_config(&ComprehensiveDeltaStrategy::set_tower_angle_offsets, cur_set.tower_angle, k, test_endstop[k].range_min, test_endstop[k].range_max, binsearch_width, cur_cartesian, target);
                    move_randomly_towards(cur_set.tower_angle[k], best_value, temp * caltype.endstop.annealing_temp_mul, target, overrun_divisor);
                }
                set_tower_angle_offsets(cur_set.tower_angle[X], cur_set.tower_angle[Y], cur_set.tower_angle[Z], false);
                

            }


            // Tell the robot to recalculate the kinematics
            post_adjust_kinematics();


            // *****************************
            // * Re-center all test ranges *
            // *****************************

            test_delta_radius.reset_min_max();
            test_arm_length.reset_min_max();
            for(k=0; k<3; k++) {
                test_endstop[k].reset_min_max();
                test_delta_radius_offset[k].reset_min_max();
                test_arm_length_offset[k].reset_min_max();
                test_angle_offset[k].reset_min_max();
            }
            

            // ****************
            // * Housekeeping *
            // ****************

            if(try_mod_10 == 0) {
                float tempE = simulate_FK_and_get_energy(test_axis, cur_set.trim, cur_cartesian);
                _printf("[HC] Try %d of %d, energy=%1.3f (want <= %1.3f)\n", annealing_try, annealing_tries, tempE, global_target);

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
                        _printf("[HC] Annealing has stalled - aborting.\n");
                        break;
                    }
                    
                    /*
                    // For debugging
                    _printf("[HC] Last energy counts: ");
                    for(j=0; j<LAST_ENERGY_N; j++) {
                        _printf("%1.3f ", last_energy[j]);
                    }
                    _printf("sigma=%1.3f\n", sigma);
                    */

                }
                
                _printf("\n");

                // Abort if within the global target
                if(tempE <= global_target) {
                    _printf("[HC] Annealing : Within target\n");
                    break;
                }
            } // try_mod_10 == 0
            
            flush();


        } // annealing_try
        
        float endE = simulate_FK_and_get_energy(test_axis, cur_set.trim, cur_cartesian);
        _printf(" \n[HC] End of annealing pass (energy=%1.3f)\n", endE);
        
        if(endE <= global_target) {
            _printf("[HC] /!\\ SUCCESS /!\\\n");
            break;
        }


        _printf(" \n");

    } // outer_try


    // Print the results
    _printf("[HC] Heuristic calibration complete (energy=%1.3f). Final settings:\n", simulate_FK_and_get_energy(test_axis, cur_set.trim, cur_cartesian));

    // Normalize trim
    auto mm = std::minmax({ cur_set.trim[X], cur_set.trim[Y], cur_set.trim[Z] });
    cur_set.trim[X] -= mm.second;
    cur_set.trim[Y] -= mm.second;
    cur_set.trim[Z] -= mm.second;
    set_trim(cur_set.trim[X], cur_set.trim[Y], cur_set.trim[Z]);

    print_kinematics();
    print_depths(cur_cartesian);

    _printf(" \n[HC] Type M500 to save.\n");

    return false;

}


// Find the most optimal configuration for a test function (e.g. set_delta_radius())
// (float version)
float ComprehensiveDeltaStrategy::find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, bool), float min, float max, float binsearch_width, float cartesian[DM_GRID_ELEMENTS][3], float target) {

    float energy_min, energy_max;
    
    // Find the direction of the most optimal configuration using a binary search
    for(int j=0; j<250; j++) {

        // Test energy at min & max

        ((this)->*test_function)(min, true);
        energy_min = simulate_FK_and_get_energy(test_axis, cur_set.trim, cartesian);

        ((this)->*test_function)(max, true);

        energy_max = simulate_FK_and_get_energy(test_axis, cur_set.trim, cartesian);

        // Who won?
        if(max - min <= target) {
            break;
        }
        if(energy_min < energy_max) {
            max -= ((max - min) * binsearch_width);
        } else {
            min += ((max - min) * binsearch_width);
        }
    
    }

    return (min + max) / 2.0f;

} 


// Find the most optimal configuration for a test function (e.g. set_delta_radius())
// (float[3] version) 
float ComprehensiveDeltaStrategy::find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, float, float, bool), float values[3], int value_idx, float min, float max, float binsearch_width, float cartesian[DM_GRID_ELEMENTS][3], float target) {

    int j;
    float energy_min, energy_max;

    // Find the direction of the most optimal configuration using a binary search
    for(j=0; j<250; j++) {

        // Test energy at min & max
        values[value_idx] = min;

        ((this)->*test_function)(values[X], values[Y], values[Z], true);
        energy_min = simulate_FK_and_get_energy(test_axis, cur_set.trim, cartesian);

        values[value_idx] = max;
        ((this)->*test_function)(values[X], values[Y], values[Z], true);

        energy_max = simulate_FK_and_get_energy(test_axis, cur_set.trim, cartesian);

        // Who won?
        if(max - min <= target) {
            break;
        }
        if(energy_min < energy_max) {
            max -= ((max - min) * binsearch_width);
        } else {
            min += ((max - min) * binsearch_width);
        }
    
    }

    return (min + max) / 2.0f;

} 


// find_optimal_config() requires a test function that takes three floats and returns a bool
bool ComprehensiveDeltaStrategy::set_test_trim(float x, float y, float z, bool dummy) {
    cur_set.trim[X] = x;
    cur_set.trim[Y] = y;
    cur_set.trim[Z] = z;
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
void ComprehensiveDeltaStrategy::simulate_IK(float cartesian[DM_GRID_ELEMENTS][3], float trim[3]) {

    for(int j = 0; j < DM_GRID_ELEMENTS; j++) {

        // Current cartesian coordinates of the depth map
        cartesian[j][X] = test_point[j][X];
        cartesian[j][Y] = test_point[j][Y];
        cartesian[j][Z] = depth_map[j].rel;
        
        // Query the robot: Where do the axes have to be for the effector to be at these coordinates?
        THEKERNEL->robot->arm_solution->cartesian_to_actuator(cartesian[j], test_axis[j]);
        
        // Adjust axis positions to simulate the effects of trim
        test_axis[j][X] += trim[X];
        test_axis[j][Y] += trim[Y];
        test_axis[j][Z] += trim[Z];
        
    }

}


// Simulate forward (actuator->cartesian) kinematics (returns the "energy" of the end result)
// The resulting cartesian coordinates are stored in cartesian[][]
float ComprehensiveDeltaStrategy::simulate_FK_and_get_energy(float axis_position[DM_GRID_ELEMENTS][3], float trim[3], float cartesian[DM_GRID_ELEMENTS][3]) {

    float trimmed[3];

    for(int j = 0; j < DM_GRID_ELEMENTS; j++) {
        if(j != TP_CTR) {
            trimmed[X] = axis_position[j][X] - trim[X];
            trimmed[Y] = axis_position[j][Y] - trim[Y];
            trimmed[Z] = axis_position[j][Z] - trim[Z];
            THEKERNEL->robot->arm_solution->actuator_to_cartesian(trimmed, cartesian[j]);
            test_depth_map[j].abs = cartesian[j][Z];
            test_depth_map[j].rel = cartesian[j][Z];
        } else {
            test_depth_map[j].abs = 0;
            test_depth_map[j].rel = 0;	// TP_CTR
        }
    }

    return calc_energy(cartesian);

}



















// Initialize test points to be used with G31 operations
void ComprehensiveDeltaStrategy::init_test_points() {

    // The grid is (2 * probe_radius) x (2 * probe_radius)
    float x, y;
    int n = 0;
    float point_spacing = (probe_radius * 2) / (DM_GRID_DIMENSION - 1);
    for(y = probe_radius; y >= -probe_radius; y-= point_spacing) {
        for(x = -probe_radius; x <= probe_radius; x += point_spacing) {
            test_point[n][X] = x;
            test_point[n][Y] = y;
            n++;
        }
    }

}


// Set the adjust function. This tells the kernel how to adjust Z for any point.
// I used ThreePointStrategy.cpp as an example.
void ComprehensiveDeltaStrategy::set_adjust_function(bool on) {

    z_compensation_enabled = on;

    if(on) {
        _printf("[ST] Depth correction enabled.\n");
        THEKERNEL->robot->compensationTransform = [this](float target[3]) { target[Z] += this->get_adjust_z(target[X], target[Y]); };
    } else {
        _printf("[ST] Depth correction disabled.\n");
        THEKERNEL->robot->compensationTransform = nullptr;
    }

}


// Figure out how far up or down we need to move the effector to conform to the print surface shape.
// This is done with a bilinear interpolation. Everything is done on the bili struct, a class member,
// so we don't have to waste time allocating variables on the stack. bili also holds calculated
// values that are used over and over, the idea being to finish the interpolation in the absolute
// minimum number of cycles possible without getting into hand-coded assembly. (Which C++ should
// compete fairly well with for speed, anyway.)
float ComprehensiveDeltaStrategy::get_adjust_z(float targetX, float targetY) {

    // Based on code retrieved from:
    // http://stackoverflow.com/questions/8808996/bilinear-interpolation-to-enlarge-bitmap-images

    // Determine which quad the point is in
    // Thx Liviu:
    // http://stackoverflow.com/questions/16592870/map-the-point-x-y-in-cartesian-coordinate-into-2d-array-elements
    // -----------------------------------------
    // The print surface is in Cartesian.
    // Our array is in single-quadrant (origin at 0,0; X grows right and Y grows down).
    // Translate surface coordinates to array coordinates by adding the difference between coordinate systems.
    // Translate point from cartesian to array coordinates

    // Constrain tested points to probe radius
// TODO: Test this & uncomment.
    targetX = clamp(targetX, -probe_radius, probe_radius);
    targetY = clamp(targetY, -probe_radius, probe_radius);

    // Calculate (floating-point) array position
    bili.array_x = (targetX - -probe_radius) * bili.cartesian_to_array_scaler;
    bili.array_y = (-targetY - -probe_radius) * bili.cartesian_to_array_scaler;	// Y inverted since it starts high and ends low

    // Calculate bounding box
    bili.x1 = floor(bili.array_x);
    bili.y1 = floor(bili.array_y);
    bili.x2 = bili.x1 + 1;
    bili.y2 = bili.y1 + 1;
//_printf("Array BB: {%1.2f, %1.2f} - {%1.2f, %1.2f} ", bili.x1, bili.y1, bili.x2, bili.y2);

    // Calculate surface transform array indices for bounding box corners
    //  x1 ____________ x2  
    // y1 | Q11    Q21
    //    | 
    //    |
    // y2 | Q12    Q22
    bili.st_Q11 = (bili.y1 * DM_GRID_DIMENSION) + bili.x1;
    bili.st_Q12 = (bili.y2 * DM_GRID_DIMENSION) + bili.x1;
    bili.st_Q21 = (bili.y1 * DM_GRID_DIMENSION) + bili.x2;
    bili.st_Q22 = (bili.y2 * DM_GRID_DIMENSION) + bili.x2;

    // Retrieve heights from the quad's points
    bili.Q11 = surface_transform[bili.st_Q11];
    bili.Q12 = surface_transform[bili.st_Q12];
    bili.Q21 = surface_transform[bili.st_Q21];
    bili.Q22 = surface_transform[bili.st_Q22];
//_printf("Heights: {%1.2f, %1.2f, %1.2f, %1.2f} ", bili.Q11, bili.Q12, bili.Q21, bili.Q22);

    // Bilinearly interpolate
    // -----------------------------------------
    // Set up the first terms
    bili.divisor = (bili.x2 - bili.x1) * (bili.y2 - bili.y1);
    bili.first_term[0] = bili.Q11 / bili.divisor;
    bili.first_term[1] = bili.Q21 / bili.divisor;
    bili.first_term[2] = bili.Q12 / bili.divisor;
    bili.first_term[3] = bili.Q22 / bili.divisor;
//_printf("first terms: %1.2f, %1.2f, %1.2f, %1.2f\n", bili.first_term[0], bili.first_term[1], bili.first_term[2], bili.first_term[3]);

    // Set up the second and third terms
    bili.x2_minus_x = bili.x2 - bili.array_x;
    bili.x_minus_x1 = bili.array_x - bili.x1;
    bili.y2_minus_y = bili.y2 - bili.array_y;
    bili.y_minus_y1 = bili.array_y - bili.y1;
//_printf("x2_minus_x: %1.2f\nx_minus_x1: %1.2f\ny2_minus_y: %1.2f y_minus_y1: %1.2f\n", bili.x2_minus_x, bili.x_minus_x1, bili.y2_minus_y, bili.y_minus_y1);

    // Interpolate    
    bili.result =
        bili.first_term[0] * bili.x2_minus_x * bili.y2_minus_y +
        bili.first_term[1] * bili.x_minus_x1 * bili.y2_minus_y +
        bili.first_term[2] * bili.x2_minus_x * bili.y_minus_y1 +
        bili.first_term[3] * bili.x_minus_x1 * bili.y_minus_y1;
//_printf(" - ");

//static unsigned int cnt;
//if(cnt++ % 100 == 0) _printf("z-adj: %1.3f\n", bili.result);

    return bili.result;

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
                _printf("[RT] Too many samples!\n");
                return false;
            }
        }
    }

    float sample[nSamples];
    if(probe_smoothing < 1) probe_smoothing = 1;
    if(probe_smoothing > 10) probe_smoothing = 10;

    // Hi
    _printf("[RT]    Repeatability test: %d samples (S)\n", nSamples);
    _printf("[RT]      Acceleration (A): %1.1f\n", want_acceleration = 0 ? THEKERNEL->planner->get_acceleration() : want_acceleration);
    _printf("[RT]    Debounce count (B): %d\n", zprobe->getDebounceCount());
    _printf("[RT]  Smooth decel (D0|D1): %s\n", zprobe->getDecelerateOnTrigger() ? "true" : "false");
    _printf("[RT] Eccentricity test (E): %s\n", do_eccentricity_test ? "on" : "off");
    _printf("[RT]   Probe smoothing (P): %d\n", probe_smoothing);
    _printf("[RT]     Probe priming (Q): %d\n", probe_priming);
    _printf("[RT]             Feedrates: Fast (U) = %1.3f, Slow (V) = %1.3f\n", zprobe->getFastFeedrate(), zprobe->getSlowFeedrate());
    _printf("[RT] 1 step = %1.5f mm.\n", zprobe->zsteps_to_mm(1.0f));
 
    // Move into position, after safely determining the true bed height
    prepare_to_probe();

    // Prime the probe (run it a number of times to get it to "settle")
    if(!prime_probe()) return false;

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
/*                
                float r = 0;
                for(long int pause = 0; pause < 1000000; pause++) {
                    r += rand();
                }
*/
        // Probe at center
        if(do_probe_at(steps, 0, 0)) {
            sample[i] = steps;
            _printf("[RT] Test %2d of %2d: Measured %d steps (%1.3f mm)\n", i + 1, nSamples, steps, zprobe->zsteps_to_mm(steps));
            if(steps > 50000) {
                _printf("[RT] Discarding result and trying again. Check probe_height.\n");
                i--;
            } else {
                mu += (float)steps;
            }
        } else {
            _printf("[RT] do_probe_at() returned false. Check probe_height.\n");
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
    _printf("[RT] Stats:\n");
    _printf("[RT]   range: %d steps (%1.4f mm)\n", max - min, zprobe->zsteps_to_mm(max - min));
    _printf("[RT]      mu: %1.3f steps (%1.3f mm)\n", mu, zprobe->zsteps_to_mm(mu));
    _printf("[RT]   sigma: %1.3f steps (%1.3f mm)\n", sigma, zprobe->zsteps_to_mm(sigma));
    _printf("[RT] Repeatability: %1.4f (add a little to be sure)\n", rep);

    if(best_probe_calibration.sigma == -1 || sigma < best_probe_calibration.sigma) {

        _printf("[RT] This is your best score so far!\n");
        best_probe_calibration.sigma = sigma;
        best_probe_calibration.range = max - min;
        best_probe_calibration.accel = want_acceleration;
        best_probe_calibration.debounce_count = zprobe->getDebounceCount();
        best_probe_calibration.decelerate = zprobe->getDecelerateOnTrigger();
        best_probe_calibration.eccentricity = do_eccentricity_test;
        best_probe_calibration.smoothing = probe_smoothing;
        best_probe_calibration.priming = probe_priming;
        best_probe_calibration.fast = zprobe->getFastFeedrate();
        best_probe_calibration.slow = zprobe->getSlowFeedrate();

    } else {

        _printf(
            "[RT] Best score so far: [sigma=%1.3f, range=%d] => accel=%f, debounce=%d, decelerate=%s, eccentricity=%s, smoothing=%d, priming=%d, fastFR=%1.3f, slowFR=%1.3f\n",
            best_probe_calibration.sigma,
            best_probe_calibration.range,
            best_probe_calibration.accel,
            best_probe_calibration.debounce_count,
            best_probe_calibration.decelerate ? "true" : "false",
            best_probe_calibration.eccentricity ? "on" : "off",
            best_probe_calibration.smoothing,
            best_probe_calibration.priming,
            best_probe_calibration.fast,
            best_probe_calibration.slow
        );

    }

    // Print evaluation
    _printf("[RT] This score is ");
    if(rep < 0.015) {
        _printf("very good!");
    } else if(rep <= 0.03) {
        _printf("average.");
    } else if(rep <= 0.04) {
        _printf("borderline.");
    } else {
        _printf("HORRIBLE.");
    }
    _printf("\n \n");

    return true;

}


// Determine which test points we're going to probe.
void ComprehensiveDeltaStrategy::determine_active_points() {

    float origin[2] = { 0, 0 };

    for(int i=0; i<DM_GRID_ELEMENTS; i++) {

        switch(surface_shape) {
            case PSS_CIRCLE:
                if(distance2D(origin, test_point[i]) <= probe_radius) {
                    active_point[i] = true;
                } else {
                    active_point[i] = false;
                }
                break;
            
            case PSS_SQUARE:
                active_point[i] = true;
                break;
        }

    }
}


// Depth-map the print surface
// Initially useful for diagnostics, but the data may be useful for doing live height corrections
// Depths are stored in depth_map (class member)
bool ComprehensiveDeltaStrategy::depth_map_print_surface(float cartesian[DM_GRID_ELEMENTS][3], _cds_dmps_result display_results) {

// This code doesn't go in this method - just leaving it here for now...
//    float permitted_deviation = 0.05f;
//    float permitted_worsening = 0.03f;
//    if(gcode->has_letter('D')) permitted_deviation = gcode->get_value('D');
//    if(gcode->has_letter('W')) permitted_worsening = gcode->get_value('W');

    int origin_steps;	// Steps from probe_height to bed surface at bed center
    int steps; 		// Steps from probe_height to bed surface at one of the test points

    determine_active_points();

    // Measure depth from probe_from_height at bed center
    prepare_to_probe();

    if(!prime_probe()) return false;

    if(do_probe_at(origin_steps, 0, 0)) {
        depth_map[TP_CTR].rel = 0;
        depth_map[TP_CTR].abs = zprobe->zsteps_to_mm(origin_steps);
        if(display_results != RESULTS_NONE) {
            _printf("[DM] Depth to bed surface at center: %d steps (%1.3f mm)\n", origin_steps, depth_map[TP_CTR].abs);
        }
    } else {
        return false;
    }

    // Measure depth from probe_height at all test points
    float best = 999;
    float worst = 0;

    // Main probing loop
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {

        // If active_points_only, we only probe the points figured out in determine_active_points(); else we probe them all
        // We don't probe TP_CTR because we already did above, in order to be able to store relative depths
        if(i != TP_CTR && active_point[i]) {

            // Run the probe
            if(!do_probe_at(steps, test_point[i][X], test_point[i][Y])) {
                _printf("[DM] do_probe_at() returned false.\n");
                return false;
            }

            // Store result in depth_map
            depth_map[i].rel = zprobe->zsteps_to_mm(origin_steps - steps);
            depth_map[i].abs = zprobe->zsteps_to_mm(steps);

            // ...And in cartesian[]
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
                _printf("[DM] Depth: %1.3fmm (%1.3fmm absolute)\n", depth_map[i].rel, depth_map[i].abs);

            }

            flush();

        }
    }

    // Show the results (pretty)
    if(display_results == RESULTS_FORMATTED) {
        print_depths(depth_map);
    }

    return true;

}


/*
 Calibrate X/Y/Z tower endstops
 - Probe center, then test points near each tower
 - Adjust each tower's trim proportional to the measured deviation
 - Back off the adjustment constant if it stays the same or gets worse
   This corrects a rare "gimbal lock" condition in which it never stops overshooting
 - Once we get an acceptable trim, normalize it
   (otherwise it will "creep down" with each successive call that keeps existing trim)
*/
bool ComprehensiveDeltaStrategy::iterative_calibration() {

    char task[] = "IC";
    char str[] = "Iterative calibration";
    print_task_with_warning(task, str);
    zero_depth_maps();

    // Initialize test points
    // ----------------------------------------------------------------------------------------
    // Towers are 60 degrees off centerline.
    // So, the quadrants look like this:
    // Q2: -xDeg, +yDeg   Q1: +xDeg, +yDeg
    // Q3: -xDeg, -yDeg   Q4: +xDeg, -yDeg
    float xDeg = 0.866025f;
    float yDeg = 0.5;

    // Points at towers (this is simple quadrant stuff)
    float tower_point[4][3];
    tower_point[TP_CTR][X] = 0;
    tower_point[TP_CTR][Y] = 0;
    tower_point[TP_X][X] = -xDeg * probe_radius;
    tower_point[TP_X][Y] = -yDeg * probe_radius;
    tower_point[TP_Y][X] =  xDeg * probe_radius;
    tower_point[TP_Y][Y] = -yDeg * probe_radius;
    tower_point[TP_Z][X] =                    0;
    tower_point[TP_Z][Y] =         probe_radius;

    // Different calibration types can be turned on and off
    // For now we only do endstops and delta radius, but other types can be added as well
    caltype.endstop.active = true;
    caltype.delta_radius.active = true;

    // This is the target accuracy. 30 microns is pretty good.
    float target = 0.03;

    for(int outer_i = 0; outer_i < 20; outer_i++) {

        // Banner preceded by line break for easy visual parsing
        _printf(" \n[IC] Iteration %d (max %d)\n", outer_i + 1, 20);
    
        // Measure the print surface

_printf("Implementation needs to be fixed.\n");
return false;

        float dummy[DM_GRID_ELEMENTS][3];
        depth_map_print_surface(dummy, RESULTS_FORMATTED);

        // Deviation for towers
        // These are measured for all calibration types
        auto tower_minmax = std::minmax({ tower_point[TP_CTR][Z], tower_point[TP_X][Z], tower_point[TP_Y][Z], tower_point[TP_Z][Z] });
        float tower_deviation = tower_minmax.second - tower_minmax.first;

        // Do we calibrate the endstops?
        if(caltype.endstop.active) {

            // ****************
            // *** ENDSTOPS ***
            // ****************
                
            // Deviation and trimscale
            static float last_deviation;
            static float trimscale;
                
            // Do we need to reset the variables?
            if(caltype.endstop.needs_reset) {
                last_deviation = 999;
                trimscale = 1.3F;
                caltype.endstop.needs_reset = false;
            }

            _printf("[ES] Endstops: Difference => %1.3f (want %1.3f", tower_deviation, target);

            // Deviation within tolerance?
            if(fabs(tower_deviation) <= target) {
                
                // Yep
                _printf(")\n[ES] Endstops are within tolerance.\n");
                caltype.endstop.in_tolerance = true;
                    
            } else {
                
                // Nope
                _printf(", out of tolerance by %1.3f)\n", tower_deviation - target);
                caltype.endstop.in_tolerance = false;
                    
                // Get trim
                float trim[3];
                if(!get_trim(trim[X], trim[Y], trim[Z])) {
                    _printf("[ES] Couldn't query trim.\n");
                    return false;
                }
                    
                // Sanity-check trim
                if(trim[X] > 0) trim[X] = 0;
                if(trim[Y] > 0) trim[Y] = 0;
                if(trim[Z] > 0) trim[Z] = 0;
                    
                // If things stayed the same or got worse, we reduce the trimscale
                if((tower_deviation >= last_deviation) && (trimscale * 0.95 >= 0.9)) {  
                    trimscale *= 0.9;
                    _printf("[ES] ~ Deviation same or worse vs. last time - reducing trim scale to %1.3f\n", trimscale);
                }
                last_deviation = tower_deviation;
                    
                // Set all towers' trims (normalized!)
                trim[X] += (tower_minmax.first - tower_point[TP_X][Z]) * trimscale;
                trim[Y] += (tower_minmax.first - tower_point[TP_Y][Z]) * trimscale;
                trim[Z] += (tower_minmax.first - tower_point[TP_Z][Z]) * trimscale;
                    
                // Correct the downward creep issue by normalizing the trim offsets
                auto mm = std::minmax({trim[X], trim[Y], trim[Z]});
                trim[X] -= mm.second;
                trim[Y] -= mm.second;
                trim[Z] -= mm.second;
                _printf("[ES] Setting endstops to {%1.3f, %1.3f, %1.3f}.\n", trim[X], trim[Y], trim[Z]);
                    
                set_trim(trim[X], trim[Y], trim[Z]);
                    
            }

        } // caltype.endstop.active
        
        
        if(caltype.delta_radius.active) {

            // ********************                
            // *** DELTA RADIUS ***
            // ********************
            float dr_factor = 2.0;
                
            // Retrieve delta radius or die trying
            float delta_radius;
            if(!get_delta_radius(delta_radius)) {
                _printf("[DR] Couldn't query delta_radius.\n");
                return false;
            }

            // Examine differences between tower depths and use this to adjust delta_radius
            float avg = (tower_point[TP_X][Z] + tower_point[TP_Y][Z] + tower_point[TP_Z][Z]) / 3.0;
            float deviation = tower_point[TP_CTR][Z] - avg;
            _printf("[DR] Delta Radius - Depths: Center=%1.3f, Tower average=%1.3f => Difference: %1.3f (want %1.3f)\n", tower_point[TP_CTR][Z], avg, deviation, target);
            _printf("[DR] Delta radius is ");

            // Deviation within tolerance?
            if(fabs(deviation) <= target) {
                
                // Yep
                _printf("within tolerance.\n");
                caltype.delta_radius.in_tolerance = true;
                    
            } else {
                
                // Nope
                _printf("out of tolerance by %1.3f.\n", deviation - target);
                caltype.delta_radius.in_tolerance = false;
                    
                _printf("[DR] Changing delta radius from %1.3f to ", delta_radius);
                delta_radius += (deviation * dr_factor);
                _printf("%1.3f\n", delta_radius);
                set_delta_radius(delta_radius);
                   
            }

        } // caltype.delta_radius.active


        // Done with ALL tasks?
        // Right now this only does the endstops & delta radius, but more can be added later.
        if(caltype.endstop.in_tolerance && caltype.delta_radius.in_tolerance) {
            _printf(" \n");
            print_kinematics();
            _printf(" \n[IC] All done! Save settings with M500.\n");
            return true;
        }


    } // for outer_i

    _printf("[IC] Maximum tries exceeded. If this is good enough, type M500 to save.\n");
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

    // Do a relative move to an elevation of probe_height
    zprobe->coordinated_move(NAN, NAN, -probe_from_height, zprobe->getFastFeedrate(), true);

}


// Enforce clean geometry
bool ComprehensiveDeltaStrategy::require_clean_geometry() {

    if(geom_dirty) {
        _printf("[EC] Geometry has been changed since last endstop/delta radius calibration - redoing.\n");
        if(!iterative_calibration()) return false;
        if(!find_bed_center_height(true)) return false;		// Reset probe_from_height, since the endstop trim may have been changed
        geom_dirty = false;
    }

    return true;

}


// Prime the probe, if set
bool ComprehensiveDeltaStrategy::prime_probe() {

    if(probe_priming) {
        int i, steps;
        _printf("[PR] Priming probe %d times.\n", probe_priming);
        for(i=0; i<probe_priming; i++) {
            if(!do_probe_at(steps, 0, 0)) {
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
        _printf("[BH] Determining the probe-from height.\n");
        zprobe->run_probe(steps, true);
        
        // Probe from height = total measured height - height required for the probe not to drag
        probe_from_height = zprobe->zsteps_to_mm(steps) - zprobe->getProbeHeight();
        zprobe->home();

    } else {
        _printf("[BH] Probe-from height = %1.3f\n", probe_from_height);
    }

    // Move to probe_from_height (relative move!)
    zprobe->coordinated_move(NAN, NAN, -probe_from_height, zprobe->getFastFeedrate(), true);
    
    // Prime the probe - this measurement is one of the most important!
    if(!prime_probe()) return false;
    
    // Move to probing offset (also relative)
    // We do these as two seperate steps because the top of a delta's build envelope is domed,
    // and we want to avoid the possibility of asking the effector to move somewhere it can't
    zprobe->coordinated_move(probe_offset_x, probe_offset_y, NAN, zprobe->getFastFeedrate(), true);

    // Now, slowly probe the depth
    save_acceleration();
    set_acceleration(probe_acceleration);
    if(!zprobe->run_probe(steps, false)) {
        restore_acceleration();
        return false;
    }
    restore_acceleration();
    mm_probe_height_to_trigger = zprobe->zsteps_to_mm(steps);
//_printf("[BH] probe_from_height (%1.3f) + mm_PHTT (%1.3f) + probe_offset_z (%1.3f)\n", probe_from_height, mm_probe_height_to_trigger, probe_offset_z);

    // Set final bed height
    bed_height = probe_from_height + mm_probe_height_to_trigger + probe_offset_z;

    // Tell the machine about the new height
    // FIXME: Endstops.cpp might have a more direct method for doing this - if so, that should be used instead!
    // -- Construct command
    char cmd[18];       // Should be enough for "M665 Z1000.12345"
    snprintf(cmd, 17, "M665 Z%1.5f", bed_height);
//    _printf("[BH] Setting bed height: ");
//    _printf(cmd);
//    _printf("\n");
    
    // -- Send command
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    THEKERNEL->conveyor->wait_for_empty_queue();

    _printf("[BH] Bed height set to %1.3f\n", bed_height);

    return true;

    /*
    _printf(
        "find_bed_center_height(): %d steps (%f mm) from probe_height (%f) to probe_offset_z (%f)\n",
        s, (float)(s / Z_STEPS_PER_MM), probe_height, probe_offset_z);
    _printf(
        "find_bed_center_height(): Bed is %f mm tall.\n", bed_height);
    */
}


// Do a probe at a specified (X, Y) location, taking probe offset into account
bool ComprehensiveDeltaStrategy::do_probe_at(int &steps, float x, float y, bool skip_smoothing) {
    // Move to location, corrected for probe offset (if any)
    zprobe->coordinated_move(x + probe_offset_x, y + probe_offset_y, NAN, zprobe->getFastFeedrate(), false);

    // Run the number of tests specified in probe_smoothing
    steps = 0;
    int smoothing, result;
    if(skip_smoothing) {
        smoothing = 1;
    } else {
        smoothing = probe_smoothing;
    }

    save_acceleration();
    set_acceleration(probe_acceleration);

    for(int i=0; i < smoothing; i++) {
        // Run the probe
        if(!zprobe->run_probe(result)) {
            if(i != 0) steps /= i;
            _printf("[DP] do_probe_at(steps, %1.3f, %1.3f) - run_probe() returned false, s=%d.\n", x + probe_offset_x, y + probe_offset_y, steps);
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

    restore_acceleration();
    
    // Average
    steps /= smoothing;

    // Sanity check
    if(steps < 100) {
        _printf("[DP] do_probe_at(): steps=%d - this is much too small - is probe_height high enough?\n", steps);
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
    saved_acceleration = THEKERNEL->planner->get_acceleration();
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


// Following are getters/setters for endstops
bool ComprehensiveDeltaStrategy::set_trim(float x, float y, float z) {

    float t[3] {x, y, z};
    bool ok = PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
//        _printf("[ES] Set trim to: X=%f Y=%f Z=%f\n", x, y, z);
    } else {
        _printf("[ES] Unable to set trim. Are endstops enabled?\n");
    }

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


// Following are getters/setters for delta geometry variables

// Arm length
bool ComprehensiveDeltaStrategy::set_arm_length(float arm_length, bool update) {

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


// Getter/setter for ALL kinematics
bool ComprehensiveDeltaStrategy::set_kinematics(KinematicSettings settings, bool update) {

    if(settings.initialized) {
        set_delta_radius(settings.delta_radius);
        set_arm_length(settings.arm_length);
        set_trim(settings.trim[X], settings.trim[Y], settings.trim[Z]);
        set_tower_radius_offsets(settings.tower_radius[X], settings.tower_radius[Y], settings.tower_radius[Z]);
        set_tower_angle_offsets(settings.tower_angle[X], settings.tower_angle[Y], settings.tower_angle[Z]);
        set_tower_arm_offsets(settings.tower_arm[X], settings.tower_arm[Y], settings.tower_arm[Z]);
        if(update) {
            post_adjust_kinematics();
        }
        return true;
    } else {
        _printf("[SK] Tried to set kinematics to uninitialized settings!\n");
        return false;
    }
}

bool ComprehensiveDeltaStrategy::get_kinematics(KinematicSettings &settings) {

    get_delta_radius(settings.delta_radius);
    get_arm_length(settings.arm_length);
    get_trim(settings.trim[X], settings.trim[Y], settings.trim[Z]);
    get_tower_radius_offsets(settings.tower_radius[X], settings.tower_radius[Y], settings.tower_radius[Z]);
    get_tower_angle_offsets(settings.tower_angle[X], settings.tower_angle[Y], settings.tower_angle[Z]);
    get_tower_arm_offsets(settings.tower_arm[X], settings.tower_arm[Y], settings.tower_arm[Z]);
    settings.initialized = true;
    return true;

}


// Print currently set kinematics
void ComprehensiveDeltaStrategy::print_kinematics() {

    KinematicSettings settings;
    get_kinematics(settings);
    print_kinematics(settings);

}

void ComprehensiveDeltaStrategy::print_kinematics(KinematicSettings settings) {

    _printf("[PG]           Arm length: %1.3f\n", settings.arm_length);
    _printf("[PG]         Delta radius: %1.3f\n", settings.delta_radius);
    _printf("[PG]      Endstop offsets: {%1.3f, %1.3f, %1.3f}\n", settings.trim[X], settings.trim[Y], settings.trim[Z]);
    _printf("[PG] Radius offsets (ABC): {%1.3f, %1.3f, %1.3f}\n", settings.tower_radius[X], settings.tower_radius[Y], settings.tower_radius[Z]);
    _printf("[PG]  Angle offsets (DEF): {%1.3f, %1.3f, %1.3f}\n", settings.tower_angle[X], settings.tower_angle[Y], settings.tower_angle[Z]);
    _printf("[PG]    Arm offsets (TUV): {%1.3f, %1.3f, %1.3f}\n", settings.tower_arm[X], settings.tower_arm[Y], settings.tower_arm[Z]);

}


// Print measured or simulated depths
void ComprehensiveDeltaStrategy::print_depths(float depths[DM_GRID_ELEMENTS][3]) {

    _cds_depths_t _depths[DM_GRID_ELEMENTS];
    
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        _depths[i].abs = 0;
        _depths[i].rel = depths[i][Z];
    }
    
    print_depths(_depths);

}

void ComprehensiveDeltaStrategy::print_depths(_cds_depths_t depths[DM_GRID_ELEMENTS]) {

    float rel_depths[DM_GRID_ELEMENTS];
    float best = 999, worst = 0;
    float mu, sigma, min, max;

    // Print header
    _printf("[PD] ");

    int i;

    // Print all depths
    int col = 0;
    for(i=0; i<DM_GRID_ELEMENTS; i++) {

        // Statistics calc requires a one-dimensional array
        rel_depths[i] = depths[i].rel;

        // Do some statistics (sign doesn't matter, only magnitude)
        if(i != TP_CTR) {
            if(fabs(depths[i].rel) < fabs(best)) {
                best = fabs(depths[i].rel);
            }

            if(fabs(depths[i].rel) > fabs(worst)) {
                worst = fabs(depths[i].rel);
            }
        }

        // Print entry (or a blank space, if the test point is turned off)
        if(active_point[i]) {
            _printf("%1.3f", depths[i].rel);
        } else {
            _printf("     ");
        }
        
        // Space or new line?
        if(col++ < DM_GRID_DIMENSION - 1) {
            _printf("   ");
        } else if(i < DM_GRID_ELEMENTS - 1) {
            col = 0;
            _printf("\n[PD]\n[PD] ");
        }

    }

    // Calculate and print statistics.
    // The difference between "best/worst" and "min/max" is that best and worst are indifferent to sign.
    calc_statistics(rel_depths, DM_GRID_ELEMENTS, mu, sigma, min, max);
    _printf("\n[PD] Best=%1.3f, worst=%1.3f, min=%1.3f, max=%1.3f, mu=%1.3f, sigma=%1.3f, energy=%1.3f\n", best, worst, min, max, mu, sigma, calc_energy(depths));

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


// Zero out depth_map and test_depth_map.
void ComprehensiveDeltaStrategy::zero_depth_maps() {

    for(int i=0; i < DM_GRID_ELEMENTS; i++) {
        depth_map[i].abs = 0;
        depth_map[i].rel = 0;
    }

    copy_depth_map(depth_map, test_depth_map);

}


// Copy a depth map to another depth map
void ComprehensiveDeltaStrategy::copy_depth_map(_cds_depths_t source[], _cds_depths_t dest[]) {

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

}


// Display active/inactive calibration types.
// The args are either-or - they shouldn't both be true.
void ComprehensiveDeltaStrategy::display_calibration_types(bool active, bool inactive) {

    char ES[] = "Endstops (P)";
    char DR[] = "Delta Radius (Q)";
    char AL[] = "Arm Length (R)";
    char TAO[] = "Tower Angle Offset (S)";
    char format[] = "[%s, mul=%1.2f] ";

    // Display active caltypes
    if(active) {

        if(caltype.endstop.active) {
            _printf(format, ES, caltype.endstop.annealing_temp_mul);
        }

        if(caltype.delta_radius.active) {
            _printf(format, DR, caltype.delta_radius.annealing_temp_mul);
        }

        if(caltype.arm_length.active) {
            _printf(format, AL, caltype.arm_length.annealing_temp_mul);
        }

        if(caltype.tower_angle.active) {
            _printf(format, TAO, caltype.tower_angle.annealing_temp_mul);
        }
        
    } // active    

    // Display inactive caltypes
    if(inactive) {

        if(!caltype.endstop.active) {
            _printf(format, ES, caltype.endstop.annealing_temp_mul);
        }

        if(!caltype.delta_radius.active) {
            _printf(format, DR, caltype.delta_radius.annealing_temp_mul);
        }

        if(!caltype.arm_length.active) {
            _printf(format, AL, caltype.arm_length.annealing_temp_mul);
        }

        if(!caltype.tower_angle.active) {
            _printf(format, TAO, caltype.tower_angle.annealing_temp_mul);
        }

    } // inactive

    _printf("\n");

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
float ComprehensiveDeltaStrategy::calc_energy(_cds_depths_t points[DM_GRID_ELEMENTS]) {

    float cartesian[DM_GRID_ELEMENTS][3];
    for(int i=0; i<DM_GRID_ELEMENTS; i++) {
        cartesian[i][X] = test_point[i][X];
        cartesian[i][Y] = test_point[i][Y];
        cartesian[i][Z] = points[i].rel;
    }
    
    return calc_energy(cartesian);

}

float ComprehensiveDeltaStrategy::calc_energy(float cartesian[DM_GRID_ELEMENTS][3]) {

    float mu = 0;
    int i = 0;

    for(int stats = 0; stats < DM_GRID_ELEMENTS; stats++) {
        if(active_point[stats]) {
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
        _printf(" ");
    }
}


// Print a banner indicating what we're working on, and what a terrible idea it would be to touch the printer in any way, shape or form
// (except the reset button)
void ComprehensiveDeltaStrategy::print_task_with_warning(char task[], char str[]) {
    _printf(" \n[%s] %s in progress. Press Reset to abort.\n", task, str);
    _printf("[%s] /!\\ DO NOT /!\\ access the SD card, press the E-stop button, use the panel, or send any commands. Doing so may cause the probe to CRASH!\n \n", task);
}


// Flush the serial buffer
void ComprehensiveDeltaStrategy::flush() {
    THEKERNEL->call_event(ON_IDLE);
}





// Dead code waiting to be flushed down the turlet below this line - cut here:
// ---8<-----------8<-----------8<-----------8<-----------8<------------8<----




// Copy depth_map to last_depth_map & zero all of depth_map
/*
void ComprehensiveDeltaStrategy::save_depth_map() {

    for(int i = 0; i < CDS_DEPTH_MAP_N_POINTS; i++) {
        last_depth_map[i].rel = depth_map[i].rel;
        last_depth_map[i].abs = depth_map[i].abs;
    }

}
*/





/* Probe the depth of points near each tower, and at the halfway points between each tower:

        1
        /\
     2 /__\ 6
      /\  /\
     /__\/__\
    3   4    5

   This pattern defines the points of a triforce, hence the name.
*/
/*
bool ComprehensiveDeltaStrategy::probe_triforce(float (&depth)[6], float &score_avg, float &score_ISM, float &PHTT) {

    // Init test points
    int triforce[6] = { TP_Z, TP_MID_ZX, TP_X, TP_MID_XY, TP_Y, TP_MID_YZ };

    int s;				// # of steps (passed by reference to probe_delta_tower, which sets it)
    int i;
    score_avg = 0;			// Score starts at 0 (perfect) - the further away it gets, the worse off we are!
    score_ISM = 0;

    // Need to get bed height in current tower angle configuration (the following method automatically refreshes mm_PHTT)
    // We're passing the current value of PHTT back by reference in case the caller cares, e.g. if they want a baseline.
    require_clean_geometry();
    prepare_to_probe();
    if(!prime_probe()) return false;
    PHTT = mm_probe_height_to_trigger;

    // This is for storing the probe results in terms of score (deviation from center height).
    // This is different from the "scores" we return, which is the average and intersextile mean of the contents of scores[].
    float score[6];

    for(i=0; i<6; i++) {
        // Probe triforce
        _printf("[PT] Probing point %d at <%1.3f, %1.3f>.\n", i, test_point[triforce[i]][X], test_point[triforce[i]][Y]);

        // Move into position and probe the depth
        // depth[i] is probed and calculated in exactly the same way that mm_probe_height_to_trigger is
        // This means that we can compare probe results from this and mm_PHTT on equal terms
        if(!do_probe_at(s, test_point[triforce[i]][X], test_point[triforce[i]][Y])) {
            return false;
        }
        depth[i] = zprobe->zsteps_to_mm(s);
        score[i] = fabs(depth[i] - mm_probe_height_to_trigger);
    }
    
    // Do some statistics
    auto mm = std::minmax({score});
    for(i=0; i<6; i++) {
    
        // Average
        score_avg += score[i];

        // Intersextile mean (ignore lowest and highest values, keep the remaining four)
        // Works similar to an interquartile mean, but more specific to our problem domain (we always have exactly 6 samples)
        // Context: http://en.wikipedia.org/wiki/Interquartile_mean
        if(score[i] != *mm.first && score[i] != *mm.second) {
            score_ISM += score[i];
        }
    }
    score_avg /= 6;
    score_ISM /= 4;

    _printf("[TQ] Probe height to trigger at bed center (PHTT) - this is the target depth: %1.3f\n", mm_probe_height_to_trigger);
    _printf("[TQ]        Current depths: {%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f}\n", depth[0], depth[1], depth[2], depth[3], depth[4], depth[5]);
    _printf("[TQ]   Delta(depth - PHTT): {%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f}\n", fabs(depth[0] - mm_probe_height_to_trigger), fabs(depth[1] - mm_probe_height_to_trigger), fabs(depth[2] - mm_probe_height_to_trigger), fabs(depth[3] - mm_probe_height_to_trigger), fabs(depth[4] - mm_probe_height_to_trigger), fabs(depth[5] - mm_probe_height_to_trigger));
    _printf("[TQ]  Score (lower=better): avg=%1.3f, ISM=%1.3f\n", score_avg, score_ISM);

    return true;

}
*/

/*
// Depth-map an imaginary line, and points perpendicular, from a tower to its opposite point
// (across the print surface), in a given number of segments
bool ComprehensiveDeltaStrategy::depth_map_segmented_line(float first[2], float second[2], unsigned char segments) {

    // Calculate vector and length
    Vector3 vec(second[X] - first[X], second[Y] - first[Y], 0);
    Vector3 vec_norm = vec.unit();
    float dist = distance2D(first, second);
    float seg_dist = dist / (float)segments;
//    _printf("Endpoints: <%1.3f, %1.3f> to <%1.3f, %1.3f>\n", first[X], first[Y], second[X], second[Y]);
//    _printf("   Vector: <%1.3f, %1.3f>; Norm: <%1.3f, %1.3f>\n", vec[0], vec[1], vec_norm[0], vec_norm[1]);
//    _printf("     Dist: %1.3f, segment dist: %1.3f\n", dist, seg_dist);


    // Measure depth from probe_height at bed center
    int steps;
    int origin_steps = 0;

    require_clean_geometry();
    prepare_to_probe();
    
    if(!prime_probe()) return false;

    if(do_probe_at(origin_steps, 0, 0)) {
        _printf("[SL] Steps from probe_from_height to bed surface at center: %d\n", origin_steps);
    } else {
        _printf("[SL] do_probe_at() returned false.\n");
        return false;
    }

    float arm_length;
    float arm_radius;
    float armX, armY, armZ;

    get_arm_length(arm_length);
    get_delta_radius(arm_radius);
    get_tower_arm_offsets(armX, armY, armZ);
//    _printf("Segments: %d\n", segments);
//    _printf("Basic - Arm length: %1.3f  Radius: %1.3f\n", arm_length, arm_radius);
//    _printf("Arm offsets: <%1.3f, %1.3f, %1.3f>\n", armX, armY, armZ);
//    _printf("Origin Z steps: %d\n", origin_steps);

    int base_depths[segments + 1][3];

    for(int i=0; i <= segments; i++) {
        //void ComprehensiveDeltaStrategy::rotate2D(float (&point)[2], float reference[2], float angle)
        float tp[2] = { first[X] + (vec_norm[X] * seg_dist * i), first[Y] + (vec_norm[Y] * seg_dist * i) };
        float tp_pos_rot[2] = { first[X] + (vec_norm[X] * seg_dist * (i + 1)), first[Y] + (vec_norm[Y] * seg_dist * (i + 1)) };
        float tp_neg_rot[2] = { first[X] + (vec_norm[X] * seg_dist * (i + 1)), first[Y] + (vec_norm[Y] * seg_dist * (i + 1)) };
        rotate2D(tp_pos_rot, tp, 90);
        rotate2D(tp_neg_rot, tp, -90);


        _printf(
            "Segment %d endpoint at <%1.3f, %1.3f> has projection <%1.3f, %1.3f> and perpendiculars <%1.3f, %1.3f> and <%1.3f, %1.3f>\n",
            i, tp[X], tp[Y],
            first[X] + (vec_norm[X] * seg_dist * (i + 1)), first[Y] + (vec_norm[Y] * seg_dist * (i + 1)),
            tp_pos_rot[X], tp_pos_rot[Y], tp_neg_rot[X], tp_neg_rot[Y]);

            
        do_probe_at(steps, tp_pos_rot[X], tp_pos_rot[Y]);
        base_depths[i][0] = steps;
        do_probe_at(steps, tp[X], tp[Y]);
        base_depths[i][1] = steps;
        do_probe_at(steps, tp_neg_rot[X], tp_neg_rot[Y]);
        base_depths[i][2] = steps;
        
        _printf("Segment %d endpoint at <%1.3f, %1.3f> - depths: pos=%1.3f, center=%1.3f, neg=%1.3f\n", i, tp[X], tp[Y], zprobe->zsteps_to_mm(origin_steps - base_depths[i][0]), zprobe->zsteps_to_mm(origin_steps - base_depths[i][1]), zprobe->zsteps_to_mm(origin_steps - base_depths[i][2]));
    }

    return true;   

}
*/

/*
                    // Find the direction of the most optimal configuration with a binary search
                    for(j=0; j<sampling_tries; j++) {
                
                        // Test energy at min & max
                        cur_set.trim[k] = t_endstop[k].min;
                        energy_min = simulate_FK_and_get_energy(test_axis, cur_set.trim, cur_cartesian);
                        
                        cur_set.trim[k] = t_endstop[k].max;
                        energy_max = simulate_FK_and_get_energy(test_axis, cur_set.trim, cur_cartesian);

                        // Who won?
                        if(t_endstop[k].max - t_endstop[k].min <= target) {
                            break;
                        }
                        if(energy_min < energy_max) {
                            t_endstop[k].max -= ((t_endstop[k].max - t_endstop[k].min) * binsearch_width);
                        } else {
                            t_endstop[k].min += ((t_endstop[k].max - t_endstop[k].min) * binsearch_width);
                        }

                    }

                    t_endstop[k].best = (t_endstop[k].min + t_endstop[k].max) / 2.0f;
                    step = ( ((float)rand() / RAND_MAX) * temp ) + 0.001;

    //_printf("[HC] Tower %d, try %3d: best=%1.3f step=%1.3f ", k, annealing_try, t_endstop[k].best, step);
                    if(t_endstop[k].best > t_endstop[k].val + target) {
                        if(t_endstop[k].val + step > t_endstop[k].best) {
                            step /= 2;
                        }
                        t_endstop[k].val += step;
                    }
                    if(t_endstop[k].best < t_endstop[k].val - target) {
                        if(t_endstop[k].val - step < t_endstop[k].best) {
                            step /= 2;
                        }
                        t_endstop[k].val -= step;
                    }
    //_printf("val=%1.3f\n", t_endstop[k].val);
*/
