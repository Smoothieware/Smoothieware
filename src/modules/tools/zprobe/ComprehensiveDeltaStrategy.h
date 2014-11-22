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

// The order and positioning here is reflected in how they're tested and printed out
// Center test point is omitted because we already know it's at {0, 0, bedht}
#define CDS_DEPTH_MAP_N_POINTS 13
enum _cds_test_point_t {
                                  TP_Z,

              TP_OPP_Y,        TP_OPP_MID_XY,        TP_OPP_X,

                     TP_MID_ZX,             TP_MID_YZ,

                                  TP_CTR,

                   TP_OPP_MID_YZ,          TP_OPP_MID_ZX,

           TP_X,                TP_MID_XY,                   TP_Y,

                                 TP_OPP_Z
};

// For depth_map_print_surface
#define RESULTS_NONE 0
#define RESULTS_UNFORMATTED 1
#define RESULTS_FORMATTED 2


// This tracks the status of an individual caltype
// It was just going to be a struct, but it's convenient to have it initialize itself
class CalibrationType {

    public:

    bool active;
    bool needs_reset;
    bool in_tolerance;

    CalibrationType() {
        active = false;
        needs_reset = true;
        in_tolerance = false;
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
//        best = 0;
//        testval = 0;
        i = 0;
//        if(_value == TEST_INIT_MIDRANGE) {
//            value = range_min + (fabs(range_max - range_min) / 2);	// Stick it in the midpoint of its range
//        } else {
//            value = _value;
//        }
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

    // Put the settings into a random state, such that new = old += random, bound by range
//    void randomize(float fraction) {
//        value = random(value, fraction, range_min, range_max);
//    }

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

struct _cds_depths_t {
    float abs;		// Absolute depth (e.g.: 12.345)
    float rel;		// Depth relative to center (e.g.: 0.345 or -0.345)
};

class ComprehensiveDeltaStrategy : public LevelingStrategy {

public:
    ComprehensiveDeltaStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){};
    ~ComprehensiveDeltaStrategy(){};
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
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

    // These hold the current and test kinematic settings
    KinematicSettings base_set;
    KinematicSettings cur_set;
    KinematicSettings test_set;
    KinematicSettings temp_set;
    KinematicSettings winning_mu;
    KinematicSettings winning_sigma;

    // These are linked lists (test_points indexes the current & last depth maps)
    // The depth maps store Z offsets from the height at bed center
    float test_point[CDS_DEPTH_MAP_N_POINTS][2];	// 1st subscript: which point; 2nd subscript: x and y
    bool active_point[CDS_DEPTH_MAP_N_POINTS];		// Which points are currently being tested
    _cds_depths_t depth_map[CDS_DEPTH_MAP_N_POINTS];	// Current collected depth for all test points
//    _cds_depths_t last_depth_map[CDS_DEPTH_MAP_N_POINTS];
    _cds_depths_t test_depth_map[CDS_DEPTH_MAP_N_POINTS];

    // This holds the axis positions generated by the inverse kinematics
    float test_axis[CDS_DEPTH_MAP_N_POINTS][3];

    // This holds all caltypes
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

    // For holding options specific to our arm solution
    BaseSolution::arm_options_t options;

    bool geom_dirty;			// Means we need to redo the endstops/delta radius

    void prepare_to_probe();
    bool prime_probe();
//    void save_depth_map();

    // Inverse and forward kinematics simulation
    void simulate_IK(float cartesian[CDS_DEPTH_MAP_N_POINTS][3], float trim[3]);
    float simulate_FK_and_get_energy(float axis_position[CDS_DEPTH_MAP_N_POINTS][3], float trim[3], float cartesian[CDS_DEPTH_MAP_N_POINTS][3]);

    bool heuristic_calibration(int annealing_tries, float max_temp, float binsearch_width, bool simulate_only, bool keep_settings, bool zero_all_offsets, float overrun_divisor);
    float find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, bool), float min, float max, float binsearch_width, float cartesian[CDS_DEPTH_MAP_N_POINTS][3], float target);
    float find_optimal_config(bool (ComprehensiveDeltaStrategy::*test_function)(float, float, float, bool), float values[3], int value_idx, float min, float max, float binsearch_width, float cartesian[CDS_DEPTH_MAP_N_POINTS][3], float target);
    bool set_test_trim(float x, float y, float z, bool dummy);
    void move_randomly_towards(float &value, float best, float temp, float target, float overrun_divisor);

    bool require_clean_geometry();
//    bool probe_triforce(float (&depth)[6], float &score_avg, float &score_ISM, float &PHTT);
    bool measure_probe_repeatability(Gcode *gcode = nullptr);
    void determine_active_points();
    bool depth_map_print_surface(bool active_points_only, unsigned char display_results, float cartesian[CDS_DEPTH_MAP_N_POINTS][3]);
//    bool depth_map_segmented_line(float first[2], float second[2], unsigned char segments);
    bool iterative_calibration();

    void set_acceleration(float a);
    void save_acceleration();
    void restore_acceleration();

    void rotate2D(float (&point)[2], float reference[2], float angle);
    
    void post_adjust_kinematics();
    void post_adjust_kinematics(float offset[3]);

    void print_depths(_cds_depths_t depths[CDS_DEPTH_MAP_N_POINTS]);
    void print_depths(float depths[CDS_DEPTH_MAP_N_POINTS][3]);
    void str_pad_left(unsigned char spaces);

    void zero_depth_maps();
    void copy_depth_map(_cds_depths_t source[], _cds_depths_t dest[]);
    
    void clear_calibration_types();
    void display_calibration_types(bool active, bool inactive);
    
    void calc_statistics(float values[], int n_values, float &mu, float &sigma, float &min, float &max);

    float calc_energy(_cds_depths_t points[CDS_DEPTH_MAP_N_POINTS]);
    float calc_energy(float cartesian[CDS_DEPTH_MAP_N_POINTS][3]);

    void midpoint(float first[2], float second[2], float (&dest)[2]);
    float distance2D(float first[2], float second[2]);
    float distance3D(float first[3], float second[3]);
    float clamp(float n, float lower, float upper);

    bool find_bed_center_height(bool reset_all = false);
    bool do_probe_at(int &steps, float x, float y, bool skip_smoothing = false);

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

    void print_kinematics();
    void print_kinematics(KinematicSettings settings);

    void print_task_with_warning(char task[], char str[]);

    void flush();

};



#endif
