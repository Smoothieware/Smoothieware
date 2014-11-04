#ifndef _COMPREHENSIVEDELTASTRATEGY
#define _COMPREHENSIVEDELTASTRATEGY

#include "BaseSolution.h"
#include "LevelingStrategy.h"

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


#include <string.h>
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
    // These are linked lists (test_points indexes the current & last depth maps)
    // The depth maps store Z offsets from the height at bed center
    float test_point[CDS_DEPTH_MAP_N_POINTS][2];	// 1st subscript: which point; 2nd subscript: x and y
    bool active_point[CDS_DEPTH_MAP_N_POINTS];		// Which points are currently being tested
    _cds_depths_t depth_map[CDS_DEPTH_MAP_N_POINTS];	// Current collected depth for all test points
    _cds_depths_t last_depth_map[CDS_DEPTH_MAP_N_POINTS];
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

    // This holds all caltypes
    struct caltype {
        CalibrationType endstops;
        CalibrationType delta_radius;
        CalibrationType arm_length;
        CalibrationType arm_length_individual;
        CalibrationType tower_radius_individual;
        CalibrationType tower_angle_individual;
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

    void prepare_to_probe();
    bool prime_probe();
    void save_depth_map();

    bool heuristic_calibration();
    bool require_clean_geometry();
    bool probe_triforce(float (&depth)[6], float &score_avg, float &score_ISM, float &PHTT);
    bool measure_probe_repeatability(Gcode *gcode = nullptr);
    void determine_active_points();
    bool depth_map_print_surface(bool active_points_only, bool display_results);
    bool depth_map_segmented_line(float first[2], float second[2], unsigned char segments);
    bool calibrate_delta_endstops(int iterations = 20, bool keep_settings = false, float override_target = 0, float override_radius = 0);
    bool calibrate_delta_radius(int iterations = 20, float override_target = 0, float override_radius = 0);

    void set_acceleration(float a);
    void save_acceleration();
    void restore_acceleration();

    void rotate2D(float (&point)[2], float reference[2], float angle);

    void print_geometry();
    
    void post_adjust_kinematics();
    void post_adjust_kinematics(float offset[3]);

    void midpoint(float first[2], float second[2], float (&dest)[2]);
    float distance(float first[2], float second[2]);

    bool geom_dirty;			// Means we need to redo the endstops/delta radius

    bool find_bed_center_height(bool reset_all = false);
    bool do_probe_at(int &steps, float x, float y, bool skip_smoothing = false);

    bool set_trim(float x, float y, float z);
    bool get_trim(float& x, float& y, float& z);

    bool set_arm_length(float arm_length);
    bool get_arm_length(float &arm_length);
    
    bool set_delta_radius(float delta_radius);
    bool get_delta_radius(float &delta_radius);

    bool set_tower_radius_offsets(float x, float y, float z);
    bool get_tower_radius_offsets(float &x, float &y, float &z);
    
    bool set_tower_angle_offsets(float x, float y, float z);
    bool get_tower_angle_offsets(float &x, float &y, float &z);

    bool set_tower_arm_offsets(float x, float y, float z);
    bool get_tower_arm_offsets(float &x, float &y, float &z);


    void flush();

};



#endif
