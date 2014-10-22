#ifndef _COMPREHENSIVEDELTASTRATEGY
#define _COMPREHENSIVEDELTASTRATEGY

#include "BaseSolution.h"
#include "LevelingStrategy.h"

#define comprehensive_delta_strategy_checksum CHECKSUM("comprehensive-delta")

// The order and positioning here is reflected in how they're tested and printed out
// Center test point is omitted because we already know it's at {0, 0, bedht}
enum _cds_test_point_t {
                                  TP_Z,

              TP_OPP_Y,        TP_OPP_MID_XY,        TP_OPP_X,

                     TP_MID_ZX,             TP_MID_YZ,
                                /* CTR */
                   TP_OPP_MID_YZ,          TP_OPP_MID_ZX,

           TP_X,                TP_MID_XY,                   TP_Y,

                                TP_OPP_Z
};


class StreamOutput;

class ComprehensiveDeltaStrategy : public LevelingStrategy
{
public:
    ComprehensiveDeltaStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe){};
    ~ComprehensiveDeltaStrategy(){};
    bool handleGcode(Gcode* gcode);
    bool handleConfig();

private:
    float bed_height;
    float probe_offset_x;
    float probe_offset_y;
    float probe_offset_z;
    float probe_radius;
    float mm_probe_height_to_trigger;	// At bed center, distance from probe_height to where probe triggers

    // For holding options specific to our arm solution
    BaseSolution::arm_options_t options;

    // These are linked lists (test_points indexes the current & last depth maps)
    // The depth maps store Z offsets from the height at bed center
    float test_point[12][2];		// 1st subscript: which point; 2nd subscript: x and y
    float cur_depth_map[12];		// Current collected depth for all test points
    float last_depth_map[12];		// Last collected depth for all test points

    bool measure_probe_repeatability(Gcode *gcode);
    bool depth_map_print_surface(bool display_results);
    bool depth_map_segmented_line(float first[2], float second[2], unsigned char segments);
    bool calibrate_delta_endstops(Gcode *gcode);
    bool calibrate_delta_radius(Gcode *gcode);

    void init();
    void rotate2D(float (&point)[2], float reference[2], float angle);
    
    void post_adjust_kinematics();
    void post_adjust_kinematics(float offset[3]);

    void midpoint(float first[2], float second[2], float (&dest)[2]);
    float distance(float first[2], float second[2]);

    bool find_bed_center_height();
    bool do_probe_at(int &steps, float x, float y);

    bool set_trim(float x, float y, float z, StreamOutput *stream);
    bool get_trim(float& x, float& y, float& z);

    bool set_delta_basic_geometry(float arm_length, float arm_radius);
    bool get_delta_basic_geometry(float &arm_length, float &arm_radius);

    bool set_tower_radius_offsets(float x, float y, float z);
    bool get_tower_radius_offsets(float &x, float &y, float &z);
    
    bool set_tower_angle_offsets(float x, float y, float z);
    bool get_tower_angle_offsets(float &x, float &y, float &z);

    bool set_tower_arm_offsets(float x, float y, float z);
    bool get_tower_arm_offsets(float &x, float &y, float &z);

    void flush();

};

#endif
