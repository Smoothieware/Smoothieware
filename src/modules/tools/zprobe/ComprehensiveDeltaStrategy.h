#ifndef _COMPREHENSIVEDELTASTRATEGY
#define _COMPREHENSIVEDELTASTRATEGY

#include "BaseSolution.h"
#include "LevelingStrategy.h"
#include "Kernel.h"
#include "StreamOutputPool.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>


#define comprehensive_delta_strategy_checksum CHECKSUM("comprehensive-delta")

// Size of the grid to be used for depth maps
#define DM_GRID_DIMENSION 5
#define DM_GRID_ELEMENTS (DM_GRID_DIMENSION * DM_GRID_DIMENSION)


// For the iterative calibration - these are the points tested
enum _cds_test_point_t {
        TP_Z,

       TP_CTR,

TP_X,		  TP_Y
};


// For storing depths
struct _cds_depths_t {
    float abs;		// Absolute depth (e.g.: 12.345)
    float rel;		// Depth relative to center (e.g.: 0.345 or -0.345)
};


// For depth_map_print_surface
enum _cds_dmps_result {
    RESULTS_NONE,
    RESULTS_UNFORMATTED,
    RESULTS_FORMATTED
};


// Print surface shape (so we can cull points outside the radius, if it's a circle)
enum _cds_print_surface_shape {
    PSS_CIRCLE,
    PSS_SQUARE
};


// This tracks the status of an individual caltype
// It was just going to be a struct, but it's convenient to have it initialize itself
class CalibrationType {

    public:

    bool active;
    bool needs_reset;
    bool in_tolerance;
    float annealing_temp_mul;

    CalibrationType() {
        active = false;
        needs_reset = true;
        in_tolerance = false;
        annealing_temp_mul = 1;
    }

};

#define _printf THEKERNEL->streams->printf

// This is for holding a test configuration for one variable, like trim for all towers or machine delta radius.
// i[] is the iterator.
class TestConfig {

    public:

    float range_min;
    float range_max;
    float i;
    float min;
    float max;

    // Initialize a test configuration
    TestConfig(float _range_min, float _range_max) {
        range_min = _range_min;
        range_max = _range_max;
        reset_min_max();
        i = 0;
    }

    // Reset min/max to their range values
    void reset_min_max() {
        min = range_min;
        max = range_max;
    }

    // Produce a random number in [ range_min .. value .. range_max ]
    // The value will be confined to be within fraction (0..1) * width
    // I would like to optimize the second rand() out, but haven't figured out how yet
    float random(float value, float fraction, float range_min, float range_max) {

        float width = (range_max - range_min) * fraction;

        float rnd;
        int dir;
        float val;
        do {
            rnd = ((float)rand() / RAND_MAX) * width;
            dir = rand() % 2;
            if(dir) {
                val = value + (rnd / 2.0f);
            } else {
                val = value - (rnd / 2.0f);
            }
        } while(val > range_max || val < range_min);
        return val;
    
    }

};


// This is for holding ALL of the kinematic settings.
class KinematicSettings {

    public:

    bool initialized;
    float delta_radius;
    float arm_length;
    float trim[3];
    float tower_radius[3];
    float tower_angle[3];
    float tower_arm[3];
    
    void init() {
        initialized = false;
        delta_radius = 0;
        arm_length = 0;
        for(int i=0; i<3; i++) {
            trim[i] = 0;
            tower_radius[i] = 0;
            tower_angle[i] = 0;
            tower_arm[i] = 0;
        }
    }

    KinematicSettings() {
        init();
    }

    KinematicSettings(float _trim[3], float _delta_radius, float _arm_length, float _tower_radius[3], float _tower_angle[3], float _tower_arm[3]) {
        delta_radius = _delta_radius;
        arm_length = _arm_length;
        for(int i=0; i<3; i++) {
            trim[i] = _trim[i];
            tower_radius[i] = _tower_radius[i];
            tower_angle[i] = _tower_angle[i];
            tower_arm[i] = _tower_arm[i];
        }
    }

    void copy_to(KinematicSettings &settings) {
        settings.delta_radius = delta_radius;
        settings.arm_length = arm_length;
        for(int i=0; i<3; i++) {
            settings.trim[i] = trim[i];
            settings.tower_radius[i] = tower_radius[i];
            settings.tower_angle[i] = tower_angle[i];
            settings.tower_arm[i] = tower_arm[i];
        }
        settings.initialized = true;

    }

};


class ComprehensiveDeltaStrategy : public LevelingStrategy {

public:
    ComprehensiveDeltaStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){};
    ~ComprehensiveDeltaStrategy(){};
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:

    // This holds all calibration types
    struct caltype {
        CalibrationType endstop;
        CalibrationType delta_radius;
        CalibrationType arm_length;
        CalibrationType tower_angle;
    } caltype;

    // This holds the best probe calibration we've been able to get, so far, in this session
    struct best_probe_calibration {
        float sigma;
        int range;
        float accel;
        int debounce_count;
        bool decelerate;
        bool eccentricity;
        int smoothing;
        int priming;
        float fast;
        float slow;
    } best_probe_calibration;

    // Bilinear interpolation values.
    // Since get_adjust_z() may be called 100s of times per second, we don't want to have to waste time
    // allocating variables on the stack each call. Similarly, there are several calculated values that
    // are used over and over again. Therefore, we do the calculations in the most efficient manner
    // possible using this struct.
    struct bili {
        // Bounding box for the depth-map quad the coordinates are within
        float x1, y1, x2, y2;
        
        // surface_transform[] indices of the quad's four points
        int st_Q11, st_Q12, st_Q21, st_Q22;

        // Depths at the quad's four points
        float Q11, Q12, Q21, Q22;

        float cartesian_to_array_scaler;

        // Cartesian point's location in the array
        float array_x;
        float array_y;

        // Divisor: (x2 - x1)(y2 - y1)
        float divisor;
        
        // First term: Height at point / (x1 - x1) * (y2 - y1)
        //           = Height at point / divisor
        float first_term[4];

        float x2_minus_x;
        float x_minus_x1;

        float y2_minus_y;
        float y_minus_y1;
        
        float result;
    } bili;

    // These hold the current and test kinematic settings
    KinematicSettings base_set;
    KinematicSettings cur_set;
    KinematicSettings test_set;
    KinematicSettings temp_set;
    KinematicSettings winning_mu;
    KinematicSettings winning_sigma;

    // For holding options specific to our arm solution
    BaseSolution::arm_options_t options;

    // Whether the printer has a round or square build surface
    _cds_print_surface_shape surface_shape; // PSS_CIRCLE or PSS_SQUARE

    // General vars
    float bed_height;
    float probe_from_height;		// Will be set to NaN during init; call find_bed_center_height() to set
    int probe_smoothing;		// Probe this many times and return the average of the results (default 1)
    int probe_priming;			// Some probes have to be test-tapped a bunch of times before they "settle"
    float probe_offset_x;
    float probe_offset_y;
    float probe_offset_z;
    float probe_radius;
    float mm_probe_height_to_trigger;	// At bed center, distance from probe_height to where probe triggers
    float saved_acceleration;
    float probe_acceleration;
    bool geom_dirty;			// Means we need to redo the endstops/delta radius
    bool z_compensation_enabled;	// Whether Z-correction is turned on

    // These are linked lists (test_points indexes the current & last depth maps)
    // The depth maps store Z offsets from the height at bed center
    float test_point[DM_GRID_ELEMENTS][2];		// 1st subscript: which point; 2nd subscript: x and y
    bool active_point[DM_GRID_ELEMENTS];		// Which points are currently being tested
    float test_axis[DM_GRID_ELEMENTS][3];		// Axis (carriage) positions generated by simulate_IK()
    float surface_transform[DM_GRID_ELEMENTS];		// For correcting Z error
    _cds_depths_t depth_map[DM_GRID_ELEMENTS];		// Current collected depth for all test points
    _cds_depths_t test_depth_map[DM_GRID_ELEMENTS];	// Depth map generated by simulate_FK_and_get_energy()

    // Inverse and forward kinematics simulation
    void simulate_IK(float cartesian[DM_GRID_ELEMENTS][3], float trim[3]);
    float simulate_FK_and_get_energy(float axis_position[DM_GRID_ELEMENTS][3], float trim[3], float cartesian[DM_GRID_ELEMENTS][3]);

    // Parallel simulated annealing methods
    bool heuristic_calibration(int annealing_tries, float max_temp, float binsearch_width, bool simulate_only, bool keep_settings, bool zero_all_offsets, float overrun_divisor);
    float find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, bool), float min, float max, float binsearch_width, float cartesian[DM_GRID_ELEMENTS][3], float target);
    float find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, float, float, bool), float values[3], int value_idx, float min, float max, float binsearch_width, float cartesian[DM_GRID_ELEMENTS][3], float target);
    bool set_test_trim(float x, float y, float z, bool dummy);
    void move_randomly_towards(float &value, float best, float temp, float target, float overrun_divisor);
    float calc_energy(_cds_depths_t points[DM_GRID_ELEMENTS]);
    float calc_energy(float cartesian[DM_GRID_ELEMENTS][3]);

    // For depth map-based Z-correction
    void init_test_points();
    void set_adjust_function(bool on);
    float get_adjust_z(float targetX, float targetY);

    // Probe calibration
    bool measure_probe_repeatability(Gcode *gcode = nullptr);

    // Determine which points are active (circular print surface requires culling a few)
    void determine_active_points();

    // Depth map the print surface
    bool depth_map_print_surface(float cartesian[DM_GRID_ELEMENTS][3], _cds_dmps_result display_results);

    // Iterative calibration
    bool iterative_calibration();

    // Probing methods
    void prepare_to_probe();
    bool prime_probe();
    bool find_bed_center_height(bool reset_all = false);
    bool do_probe_at(int &steps, float x, float y, bool skip_smoothing = false);

    // Kinematics
    void print_kinematics();
    void print_kinematics(KinematicSettings settings);

    void post_adjust_kinematics();
    void post_adjust_kinematics(float offset[3]);

    bool set_trim(float x, float y, float z);
    bool get_trim(float& x, float& y, float& z);

    bool set_arm_length(float arm_length, bool update = true);
    bool get_arm_length(float &arm_length);
    
    bool set_delta_radius(float delta_radius, bool update = true);
    bool get_delta_radius(float &delta_radius);

    bool set_tower_radius_offsets(float x, float y, float z, bool update = true);
    bool get_tower_radius_offsets(float &x, float &y, float &z);
    
    bool set_tower_angle_offsets(float x, float y, float z, bool update = true);
    bool get_tower_angle_offsets(float &x, float &y, float &z);

    bool set_tower_arm_offsets(float x, float y, float z, bool update = true);
    bool get_tower_arm_offsets(float &x, float &y, float &z);

    bool set_kinematics(KinematicSettings settings, bool update = true);
    bool get_kinematics(KinematicSettings &settings);

    void set_acceleration(float a);
    void save_acceleration();
    void restore_acceleration();

    // Math
    void calc_statistics(float values[], int n_values, float &mu, float &sigma, float &min, float &max);
    void rotate2D(float (&point)[2], float reference[2], float angle);
    float distance2D(float first[2], float second[2]);
    float distance3D(float first[3], float second[3]);
    float clamp(float n, float lower, float upper);
    void midpoint(float first[2], float second[2], float (&dest)[2]);

    // Utilities
    void zero_depth_maps();
    void copy_depth_map(_cds_depths_t source[], _cds_depths_t dest[]);
    void clear_calibration_types();
    void display_calibration_types(bool active, bool inactive);
    void print_depths(_cds_depths_t depths[DM_GRID_ELEMENTS]);
    void print_depths(float depths[DM_GRID_ELEMENTS][3]);
    void str_pad_left(unsigned char spaces);
    bool require_clean_geometry();
    void print_task_with_warning(char task[], char str[]);
    void flush();

};



#endif
