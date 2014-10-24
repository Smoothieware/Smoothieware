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

#include <tuple>
#include <algorithm>

// probe_radius is "deprecated" in favor of just radius, but it shouldn't be.
// Using just "radius" sounds like the printer radius, but probing can't always be done that far out.
#define probe_radius_checksum CHECKSUM("probe_radius")

#define probe_offset_x_checksum  CHECKSUM("probe_offset_x")
#define probe_offset_y_checksum  CHECKSUM("probe_offset_y")
#define probe_offset_z_checksum  CHECKSUM("probe_offset_z")

#define X 0
#define Y 1

#define _printf THEKERNEL->streams->printf


bool ComprehensiveDeltaStrategy::handleConfig() {

    // default is probably wrong
    float r= THEKERNEL->config->value(leveling_strategy_checksum, comprehensive_delta_strategy_checksum, probe_radius_checksum)->by_default(-1)->as_number();
    if(r == -1) {
        // deprecated config syntax]
        r =  THEKERNEL->config->value(zprobe_checksum, probe_radius_checksum)->by_default(100.0F)->as_number();
    }
    this->probe_radius= r;

    // Effector coordinates when probe is at bed center, at the exact height where it triggers.
    // To determine this:
    // - Heat the extruder
    // - Jog it down to the print surface, so it leaves a little dot
    // - Deploy the probe and move it until its trigger is touching the dot
    // - Record the values in config as probe_offset_x/y/z
    this->probe_offset_x = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_x_checksum)->by_default(0)->as_number();
    this->probe_offset_y = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_y_checksum)->by_default(0)->as_number();
    this->probe_offset_z = THEKERNEL->config->value(comprehensive_delta_strategy_checksum, probe_offset_z_checksum)->by_default(0)->as_number();

    // Initialize test points
    // ----------------------------------------------------------------------------------------
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

    return true;

}


bool ComprehensiveDeltaStrategy::handleGcode(Gcode *gcode) {

    if( gcode->has_g) {
        // G code processing
        if(gcode->g == 29) { // Test the Z-probe for repeatability

            measure_probe_repeatability(gcode);
            return true;

        }

        if(gcode->g == 31) { // Depth-map the bed and display the results

            _printf("Depth-mapping the bed. Please stand by...\n");
            //depth_map_print_surface(true);	// Setting "true" makes it nicely print the results over serial
            //depth_map_segmented_line(test_point[TP_Y], test_point[TP_OPP_Y], 10);
            heuristic_calibration();
            return true;

        }

        if(gcode->g == 32) { // Auto calibration for delta, Z bed mapping for cartesian
            // first wait for an empty queue i.e. no moves left
            _printf("COMPREHENSIVE DELTA STRATEGY\n");
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

            if(!gcode->has_letter('R')) {
                if(!calibrate_delta_endstops(gcode)) {
                    _printf("Calibration failed to complete, probe not triggered\n");
                    return true;
                }
            }
            if(!gcode->has_letter('E')) {
                if(!calibrate_delta_radius(gcode)) {
                    _printf("Calibration failed to complete, probe not triggered\n");
                    return true;
                }
            }
            _printf("Calibration complete, save settings with M500\n");
            return true;
        }

    } else if(gcode->has_m) {
        // handle mcodes
    }

    return false;

}


// Measure probe tolerance (repeatability)
// Things that may improve repeatability:
// - feedrate
// - debounce_count
// - controller cooling, especially the stepper drivers
// - noise from other wiring in the chassis
bool ComprehensiveDeltaStrategy::measure_probe_repeatability(Gcode *gcode) {
    // Statistical variables
    int i;
    int steps;
    int nSamples = 10;

    float mu = 0;	// Mean
    float sigma = 0;	// Standard deviation
    float dev = 0;	// Sample deviation

    // Setup for number of samples / eccentricity testing
    bool do_eccentricity_test = true;
    if(gcode->has_letter('S')) {
        nSamples = (int)gcode->get_value('S');
        if(nSamples > 30) {
            _printf("[RT] Too many samples!\n");
            return false;
        }
    }
    float sample[nSamples];

    if(gcode->has_letter('E')) do_eccentricity_test = false;


    // Hi
    _printf("[RT] Repeatability test: %d samples, eccentricity test is ", nSamples);
    if(do_eccentricity_test) {
        _printf("on.\n");
    } else {
        _printf("off.\n");
    }
    _printf("[RT] 1 step = %1.5f mm.\n", zprobe->zsteps_to_mm(1.0f));
 
    // Move into position
    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), false);
    
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
            _printf("[RT] Test %d of %d: Measured %d steps (%1.3f mm)\n", i + 1, nSamples, steps, zprobe->zsteps_to_mm(steps));
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

    // Print evaluation
    _printf("[RT] This score is ");
    if(rep < 0.015) {
        _printf("very good!");
    } else if(rep <= 0.03) {
        _printf("average.");
    } else if(rep <= 0.04) {
        _printf("borderline.");
    } else {
        _printf("UNUSABLE! Please fix!");
    }
    _printf("\n \n");
    return true;

}


// calculate the X and Y positions for the three towers given the radius from the center
static std::tuple<float, float, float, float, float, float> getCoordinates(float radius) {

    float px = 0.866F * radius; // ~sin(60)
    float py = 0.5F * radius; // cos(60)
    float t1x = -px, t1y = -py; // X Tower
    float t2x = px, t2y = -py; // Y Tower
    float t3x = 0.0F, t3y = radius; // Z Tower
    return std::make_tuple(t1x, t1y, t2x, t2y, t3x, t3y);

}


// Rotate a point around another point in 2-space.
// Adapted from http://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
void ComprehensiveDeltaStrategy::rotate2D(float (&point)[2], float reference[2], float angle) {

    float s = sin(angle * 3.141595 / 180.0);
    float c = cos(angle * 3.141595 / 180.0);
    
    point[X] -= reference[X];
    point[Y] -= reference[Y];

//    out[X_AXIS] = cos * in[X_AXIS] - sin * in[Y_AXIS];
//    out[Y_AXIS] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    float xNew = point[X] * c - point[Y] * s;
    float yNew = point[X] * s + point[Y] * c;

    point[X] = xNew + reference[X];
    point[Y] = yNew + reference[Y];

//    out[X_AXIS] = cos * in[X_AXIS] - sin * in[Y_AXIS];
//    out[Y_AXIS] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    


}


// Calculate the midpoint of a 2-D line.
// first[] and second[] are floats. Resulting midpoint stored in dest[].
void ComprehensiveDeltaStrategy::midpoint(float first[2], float second[2], float (&dest)[2]) {

    dest[0] = (first[0] + second[0]) / 2;
    dest[1] = (first[1] + second[1]) / 2;

}









bool ComprehensiveDeltaStrategy::heuristic_calibration() {

    float depth[6];
    float score_avg;
    float score_ISM;
    float PHTT;
    
    probe_triforce(depth, score_avg, score_ISM, PHTT);
    
    return true;

}




/* Probe the depth of points near each tower, and at the halfway points between each tower:

        1
        /\
     2 /__\ 6
      /\  /\
     /__\/__\
    3   4    5

   This pattern defines the points of a triforce, hence the name.
*/
bool ComprehensiveDeltaStrategy::probe_triforce(float (&depth)[6], float &score_avg, float &score_ISM, float &PHTT) {

    // Init test points
    int triforce[6] = { TP_Z, TP_MID_ZX, TP_X, TP_MID_XY, TP_Y, TP_MID_YZ };

    int s;				// # of steps (passed by reference to probe_delta_tower, which sets it)
    int i;
    score_avg = 0;			// Score starts at 0 (perfect) - the further away it gets, the worse off we are!
    score_ISM = 0;

    // Need to get bed height in current tower angle configuration (the following method automatically refreshes mm_PHTT)
    // We're passing the current value of PHTT back by reference in case the caller cares, e.g. if they want a baseline.
    find_bed_center_height();
    PHTT = mm_probe_height_to_trigger;
    zprobe->home();

    // This is for storing the probe results in terms of score (deviation from center height).
    // This is different from the "scores" we return, which is the average and intersextile mean of the contents of scores[].
    float score[6];

    for(i=0; i<6; i++) {
        // Probe triforce
        _printf("[PT] Probing point %d at <%1.3f, %1.3f>.\n", i, test_point[triforce[i]][X], test_point[triforce[i]][Y]);

        // Move into position and probe the depth
        // depth[i] is probed and calculated in exactly the same way that mm_probe_height_to_trigger is
        // This means that we can compare probe results from this and mm_PHTT on equal terms
        zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate());
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















/*

 Depth-map the print surface
 Initially useful for diagnostics, but the data may be useful for doing live height corrections

*/
bool ComprehensiveDeltaStrategy::depth_map_print_surface(bool display_results) {

// This code doesn't go in this method - just leaving it here for now...
//    float permitted_deviation = 0.05f;
//    float permitted_worsening = 0.03f;
//    if(gcode->has_letter('D')) permitted_deviation = gcode->get_value('D');
//    if(gcode->has_letter('W')) permitted_worsening = gcode->get_value('W');

    // Determine bed height (this is our central reference point)
    if(!find_bed_center_height()) {
        _printf("find_bed_center_height() failed.\n");
        return false;
    }

    int origin_steps;	// Steps from probe_height to bed surface at bed center
    int steps; 		// Steps from probe_height to bed surface at one of the test points

    // Measure depth from probe_height at bed center
    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), false);
    if(do_probe_at(origin_steps, 0, 0)) {
        _printf("[DM] Steps from probe_height to bed surface at center: %d\n", origin_steps);
    } else {
        _printf("[DM] do_probe_at() returned false.\n");
        return false;
    }

    // Measure depth from probe_height at all test points
    float best = 999;
    float worst = 0;
//    float mu;
//    float sigma;
    unsigned char lines[] = { 1, 3, 2, 2, 3, 1 };
    unsigned char line = 0;
    for(int i=0; i<12; i++) {
//        _printf("Test point %d: <%1.3f, %1.3f>: ", i, test_point[i][0], test_point[i][1]);
        if(!do_probe_at(steps, test_point[i][0], test_point[i][1])) {
            _printf("[DM] do_probe_at() returned false.\n");
            return false;
        }
//        _printf("%d steps (deflection = %d steps, %1.3f mm)\n", s, origin_steps - s, zprobe->zsteps_to_mm(origin_steps - s));
        cur_depth_map[i] = origin_steps - steps;
        if(fabs(cur_depth_map[i]) < best) {
            best = cur_depth_map[i];
        }
        if(fabs(cur_depth_map[i]) > worst) {
            worst = cur_depth_map[i];
        }
        
        if(line == 0) {
            _printf("[DM] ");
        }
        
        _printf(" %1.3f ", zprobe->zsteps_to_mm(cur_depth_map[i]));
        
        if(--lines[line] <= 0) {
            line++;
            _printf("\n[DM] ");
        }

        flush();
    }
    flush();

    // Do stats
    _printf("Deviation: Best = %1.3f, Worst = %1.3f\n", zprobe->zsteps_to_mm(best), zprobe->zsteps_to_mm(worst));


    return true;

}


// Distance between two points in 2-space
float ComprehensiveDeltaStrategy::distance(float first[2], float second[2]) {
    return sqrt(pow(second[X] - first[X], 2) + pow(second[Y] - first[Y], 2));
}


// Depth-map an imaginary line from a tower to its opposite point in a given number of segments
bool ComprehensiveDeltaStrategy::depth_map_segmented_line(float first[2], float second[2], unsigned char segments) {

    // Calculate vector and length
    Vector3 vec(second[X] - first[X], second[Y] - first[Y], 0);
    Vector3 vec_norm = vec.unit();
    float dist = distance(first, second);
    float seg_dist = dist / (float)segments;
//    _printf("Endpoints: <%1.3f, %1.3f> to <%1.3f, %1.3f>\n", first[X], first[Y], second[X], second[Y]);
//    _printf("   Vector: <%1.3f, %1.3f>; Norm: <%1.3f, %1.3f>\n", vec[0], vec[1], vec_norm[0], vec_norm[1]);
//    _printf("     Dist: %1.3f, segment dist: %1.3f\n", dist, seg_dist);


    // Determine bed height (this is our central reference point)
    if(!find_bed_center_height()) {
        _printf("find_bed_center_height() failed.\n");
        return false;
    }

    // Measure depth from probe_height at bed center
    int steps;
    int origin_steps = 0;

    zprobe->home();

    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), false);

    if(do_probe_at(origin_steps, 0, 0)) {
        _printf("[SL] Steps from probe_height to bed surface at center: %d\n", origin_steps);
    } else {
        _printf("[SL] do_probe_at() returned false.\n");
        return false;
    }

    float arm_length;
    float arm_radius;
    float armX, armY, armZ;

    get_delta_basic_geometry(arm_length, arm_radius);
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

/*
        _printf(
            "Segment %d endpoint at <%1.3f, %1.3f> has projection <%1.3f, %1.3f> and perpendiculars <%1.3f, %1.3f> and <%1.3f, %1.3f>\n",
            i, tp[X], tp[Y],
            first[X] + (vec_norm[X] * seg_dist * (i + 1)), first[Y] + (vec_norm[Y] * seg_dist * (i + 1)),
            tp_pos_rot[X], tp_pos_rot[Y], tp_neg_rot[X], tp_neg_rot[Y]);
*/
            
        do_probe_at(steps, tp_pos_rot[X], tp_pos_rot[Y]);
        base_depths[i][0] = steps;
        do_probe_at(steps, tp[X], tp[Y]);
        base_depths[i][1] = steps;
        do_probe_at(steps, tp_neg_rot[X], tp_neg_rot[Y]);
        base_depths[i][2] = steps;
        
        _printf("Segment %d endpoint at <%1.3f, %1.3f> - depths: pos=%1.3f, center=%1.3f, neg=%1.3f\n", i, tp[X], tp[Y], zprobe->zsteps_to_mm(origin_steps - base_depths[i][0]), zprobe->zsteps_to_mm(origin_steps - base_depths[i][1]), zprobe->zsteps_to_mm(origin_steps - base_depths[i][2]));
    }

/*
_printf("\n *** Shortening Y's arm by 1\n");

    // Try shortening the arm
    armY = -1;
    set_tower_arm_offsets(armX, armY, armZ);
    post_adjust_kinematics();
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), false);

    for(int i=0; i <= segments; i++) {
        do_probe_at(steps, first[X] + (vec_norm[X] * seg_dist * i), first[Y] + (vec_norm[Y] * seg_dist * i));
        int diff = base_depths[i] - steps;
        _printf("Segment %d endpoint at <%1.3f, %1.3f> has depth %1.3f (change: %1.3f)\n", i, first[X] + (vec_norm[X] * seg_dist * i), first[Y] + (vec_norm[Y] * seg_dist * i), zprobe->zsteps_to_mm(origin_steps - steps), zprobe->zsteps_to_mm(diff));
    }

_printf("\n *** Lengthening Y's arm by 1\n");

    // Try lengthening the arm
    armY = 1;
    set_tower_arm_offsets(armX, armY, armZ);
    post_adjust_kinematics();
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getFastFeedrate(), false);


    for(int i=0; i <= segments; i++) {
        do_probe_at(steps, first[X] + (vec_norm[X] * seg_dist * i), first[Y] + (vec_norm[Y] * seg_dist * i));
        int diff = base_depths[i] - steps;
        _printf("Segment %d endpoint at <%1.3f, %1.3f> has depth %1.3f (change: %1.3f)\n", i, first[X] + (vec_norm[X] * seg_dist * i), first[Y] + (vec_norm[Y] * seg_dist * i), zprobe->zsteps_to_mm(origin_steps - steps), zprobe->zsteps_to_mm(diff));
    }
*/


// STRATEGY

// Perpendicular Probing




    return true;   
}


/*
 Level X/Y/Z tower endstops
 - Probe center, then test points near each tower
 - Adjust each tower's trim proportional to the measured deviation
 - Back off the adjustment constant if it stays the same or gets worse
   This corrects a rare "gimbal lock" condition in which it never stops overshooting
 - Once we get an acceptable trim, normalize it
   (otherwise it will "creep down" with each successive call that keeps existing trim)
*/
bool ComprehensiveDeltaStrategy::calibrate_delta_endstops(Gcode *gcode) {

    float target = 0.03F;
    if(gcode->has_letter('I')) target = gcode->get_value('I'); // override default target
    if(gcode->has_letter('J')) this->probe_radius = gcode->get_value('J'); // override default probe radius

    bool keep = false;
    if(gcode->has_letter('K')) keep = true;   // keep current settings

    float deviation = 0;                                       // Stores current deviation from center
    float last_deviation = 999;                                // Stores last deviation from center   

    _printf(" \n");
    _printf("[ES] Calibrating endstops: target %fmm, radius %fmm\n", target, this->probe_radius);

    // How much to try adjusting the trim each time (empirically determined, old default was 1.2522)
    // If an adjustment makes the calibration worse, this will be reduced unless it's already < 1.0 
    float trimscale = 1.3F;                                     // Starting point

    // Set probe points
    float t1x, t1y, t2x, t2y, t3x, t3y;
    std::tie(t1x, t1y, t2x, t2y, t3x, t3y) = getCoordinates(this->probe_radius);

    // Set up trim
    float trimx = 0.0F, trimy = 0.0F, trimz = 0.0F;
    if(!keep) {

        // Zero trim values
        if(!set_trim(0, 0, 0, gcode->stream)) return false;

    } else {

        // Get current trim, and continue from that
        if (get_trim(trimx, trimy, trimz)) {
            _printf("[ES] Current Trim: x=%1.3f, y=%1.3f, z=%1.3f\r\n", trimx, trimy, trimz);
        } else {
            _printf("[ES] Couldn't get current trim. Are endstops enabled?\n");
            return false;
        }

    }

    // Find bed height
    int s;
    if(!find_bed_center_height()) {
        _printf("find_bed_center_height() failed.\n");
        return false;
    }

    zprobe->home();

    // Do a relative move from home to the point above the bed
    zprobe->coordinated_move(NAN, NAN, -bed_height + zprobe->getProbeHeight(), zprobe->getFastFeedrate(), true);

    // Get initial probes
    // ========================================================================
    // Probe the base of the X tower
    if(!do_probe_at(s, t1x, t1y)) return false;
    float t1z = zprobe->zsteps_to_mm(s);

    // Probe the base of the Y tower
    if(!do_probe_at(s, t2x, t2y)) return false;
    float t2z = zprobe->zsteps_to_mm(s);

    // Probe the base of the Z tower
    if(!do_probe_at(s, t3x, t3y)) return false;
    float t3z = zprobe->zsteps_to_mm(s);

    // Is the trim already within spec? If so, we're done
    auto mm = std::minmax({t1z, t2z, t3z});
    if((mm.second - mm.first) <= target) { 
        _printf("[ES] Trim already set within required parameters: difference = %f\n", mm.second - mm.first);
        return true;
    } else {
        _printf("[ES] Towers out of spec by %1.3f - will need to level the endstops\n", (mm.second - mm.first) - target);
        last_deviation = mm.second - mm.first;
    }

    // Set all towers' trims to worst case (we need to start with negative trim; positive = belt grinding!)
    trimx += (mm.first - t1z) * trimscale;
    trimy += (mm.first - t2z) * trimscale;
    trimz += (mm.first - t3z) * trimscale;
    
    
    // Main endstop leveling loop
    // ========================================================================
    for (int i = 1; i <= 20; ++i) {

        // Flush serial buffer
        THEKERNEL->call_event(ON_IDLE);

        // Tell the robot what the new trim is
        if(!set_trim(trimx, trimy, trimz, gcode->stream)) return false;
        zprobe->home();

        // Move probe to start position at probe_height millimeters above the bed (relative move)
        zprobe->coordinated_move(NAN, NAN, -bed_height + zprobe->getProbeHeight(), zprobe->getFastFeedrate(), true);

        // probe the base of the X tower
        if(!do_probe_at(s, t1x, t1y)) return false;
        t1z = zprobe->zsteps_to_mm(s);

        // probe the base of the Y tower
        if(!do_probe_at(s, t2x, t2y)) return false;
        t2z = zprobe->zsteps_to_mm(s);

        // probe the base of the Z tower
        if(!do_probe_at(s, t3x, t3y)) return false;
        t3z = zprobe->zsteps_to_mm(s);

        // Is this part of the calibration good enough to move on?
        mm = std::minmax({t1z, t2z, t3z});
        deviation = mm.second - mm.first; 

        if(deviation > target) {

            // Not close enough yet, adjust trim on all towers
            _printf("[ES] ~ Towers still out of spec by %1.3f\n", (mm.second - mm.first) - target);

            // If things stayed the same or got worse, we reduce the trimscale
            if((deviation >= last_deviation) && (trimscale * 0.95 >= 0.9)) {  
                trimscale *= 0.9;
                _printf("[ES] ~ Deviation same or worse vs. last time - reducing trim scale to %1.3f\n", trimscale);
            }
            last_deviation = deviation;

            // Set all towers' trims
            trimx += (mm.first - t1z) * trimscale;
            trimy += (mm.first - t2z) * trimscale;
            trimz += (mm.first - t3z) * trimscale;

        } else {

            _printf("[ES] Trim set to within required paramters: want %1.3f, difference is %1.3f\n", target, mm.second - mm.first);

            // Correct the downward creep issue by normalizing the trim offsets.
            mm = std::minmax({trimx, trimy, trimz});
            _printf("[ES] Trim is {%1.3f, %1.3f, %1.3f} - normalizing... ", trimx, trimy, trimz);
            trimx -= mm.second;
            trimy -= mm.second;
            trimz -= mm.second;

            // Tell robot about new trim
            if(!set_trim(trimx, trimy, trimz, gcode->stream)) {
                _printf("[ES] ERROR: Couldn't normalize trim\n");
                return false;
            } else {
                _printf("[ES] New values: {%1.3f, %1.3f, %1.3f}.\n \n", trimx, trimy, trimz);
                return true;
            }
        }
    }

    if((mm.second - mm.first) > target) {
        _printf("[ES] Trim did not resolve to within required parameters: delta %f\n", mm.second - mm.first);
        return false;
    }

    // Flush serial buffer
    THEKERNEL->call_event(ON_IDLE);

    return true;
}


/*
    probe edges to get outer positions, then probe center
    modify the delta radius until center and X converge
*/

bool ComprehensiveDeltaStrategy::calibrate_delta_radius(Gcode *gcode) {

    float target = 0.03F;
    if(gcode->has_letter('I')) target = gcode->get_value('I'); // override default target
    if(gcode->has_letter('J')) this->probe_radius = gcode->get_value('J'); // override default probe radius

    _printf("[DR] Calibrating delta radius: target %f, radius %f\n", target, this->probe_radius);

    // get probe points
    float t1x, t1y, t2x, t2y, t3x, t3y;
    std::tie(t1x, t1y, t2x, t2y, t3x, t3y) = getCoordinates(this->probe_radius);

    zprobe->home();
    // find bed, then move to a point 5mm above it
    int s;
    if(!zprobe->run_probe(s, true)) return false;
    float bedht = zprobe->zsteps_to_mm(s) - zprobe->getProbeHeight(); // distance to move from home to probe_height above bed
    _printf("[DR] Probe trigger height is %f mm\n", bedht);

    zprobe->home();
    zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // do a relative move from home to the point above the bed

    // probe center to get reference point at this Z height
    int dc;
    if(!do_probe_at(dc, 0, 0)) return false;
    _printf("[DR] Center Z: %1.3fmm (%d steps)\n", zprobe->zsteps_to_mm(dc), dc);
    float cmm = zprobe->zsteps_to_mm(dc);

    // get current delta radius
    float delta_radius = 0.0F;
    BaseSolution::arm_options_t options;
    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        delta_radius = options['R'];
    }
    if(delta_radius == 0.0F) {
        _printf("[DR] ERROR: Delta radius not set in config! Is this a delta?\n");
        return false;
    }
    options.clear();

    float drinc = 2.5F; // approx
    for (int i = 1; i <= 10; ++i) {
        // probe t1, t2, t3 and get average, but use coordinated moves, probing center won't change
        int dx, dy, dz;
        if(!do_probe_at(dx, t1x, t1y)) return false;
        _printf("T1-%d Z: %1.3fmm (%d steps)\n", i, zprobe->zsteps_to_mm(dx), dx);
        if(!do_probe_at(dy, t2x, t2y)) return false;
        _printf("T2-%d Z: %1.3fmm (%d steps)\n", i, zprobe->zsteps_to_mm(dy), dy);
        if(!do_probe_at(dz, t3x, t3y)) return false;
        _printf("T3-%d Z: %1.3fm (%d steps)\n", i, zprobe->zsteps_to_mm(dz), dz);

        // now look at the difference and reduce it by adjusting delta radius
        float m = zprobe->zsteps_to_mm((dx + dy + dz) / 3.0F);
        float d = cmm - m;
        _printf("C-%d Z-ave:%1.4f delta: %1.3f\n", i, m, d);

        if(abs(d) <= target) break; // resolution of success

        // increase delta radius to adjust for low center
        // decrease delta radius to adjust for high center
        delta_radius += (d * drinc);

        // set the new delta radius
        options['R'] = delta_radius;
        THEKERNEL->robot->arm_solution->set_optional(options);
        _printf("Setting delta radius to: %1.4f\n", delta_radius);

        zprobe->home();
        zprobe->coordinated_move(NAN, NAN, -bedht, zprobe->getFastFeedrate(), true); // needs to be a relative coordinated move

        // flush the output
        THEKERNEL->call_event(ON_IDLE);
    }
    return true;
}



bool ComprehensiveDeltaStrategy::set_trim(float x, float y, float z, StreamOutput *stream) {

    float t[3] {x, y, z};
    bool ok = PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
        stream->printf("[ES] Set trim to: X=%f Y=%f Z=%f\n", x, y, z);
    } else {
        stream->printf("[ES] Unable to set trim. Are endstops enabled?\n");
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

/*
    options['L'] = this->arm_length;
    options['R'] = this->arm_radius;
    options['A'] = this->tower1_offset;
    options['B'] = this->tower2_offset;
    options['C'] = this->tower3_offset;
    options['D'] = this->tower1_angle;
    options['E'] = this->tower2_angle;
    options['F'] = this->tower3_angle;
    options['G'] = this->tower1_arm_offset;
    options['H'] = this->tower2_arm_offset;
    options['I'] = this->tower3_arm_offset;

*/

// When delta parameters are adjusted, you have to either home the printer or reset the kinematics.
// If you don't, there will be a violent jerk the next time you ask the robot to move! This routine
// should save a LOT of time over homing the robot. NOTE: Use the version of this method with offsets
// if you reset the endstops because their offset values ARE NOT used in motion planning!
void ComprehensiveDeltaStrategy::post_adjust_kinematics() {

    float pos[3];
    THEKERNEL->robot->get_axis_position(pos);
    THEKERNEL->robot->reset_axis_position(pos[0], pos[1], pos[2]);

}

// This is the version you want to use if you're fiddling with the endstops. Note that endstop
// offset values are POSITIVE (steps down, not up).
void ComprehensiveDeltaStrategy::post_adjust_kinematics(float offset[3]) {

    float pos[3];
    THEKERNEL->robot->get_axis_position(pos);
    THEKERNEL->robot->reset_axis_position(pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]);

}

bool ComprehensiveDeltaStrategy::set_delta_basic_geometry(float arm_length, float arm_radius) {
    options['L'] = arm_length;
    options['R'] = arm_radius;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        post_adjust_kinematics();
        return true;
    } else {
        return false;
    }
}

bool ComprehensiveDeltaStrategy::get_delta_basic_geometry(float &arm_length, float &arm_radius) {
    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        arm_length = options['L'];
        arm_radius = options['R'];
        return true;
    } else {
        return false;
    }
}


bool ComprehensiveDeltaStrategy::set_tower_radius_offsets(float x, float y, float z) {
    options['A'] = x;
    options['B'] = y;
    options['C'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        post_adjust_kinematics();
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

              
bool ComprehensiveDeltaStrategy::set_tower_angle_offsets(float x, float y, float z) {
    options['D'] = x;
    options['E'] = y;
    options['F'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        post_adjust_kinematics();
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

                          
bool ComprehensiveDeltaStrategy::set_tower_arm_offsets(float x, float y, float z) {
    options['G'] = x;
    options['H'] = y;
    options['I'] = z;
    if(THEKERNEL->robot->arm_solution->set_optional(options)) {
        post_adjust_kinematics();
        return true;
    } else {
        return false;
    }
}

bool ComprehensiveDeltaStrategy::get_tower_arm_offsets(float &x, float &y, float &z) {
    if(THEKERNEL->robot->arm_solution->get_optional(options)) {
        x = options['G'];
        y = options['H'];
        z = options['I'];
        return true;
    } else {
        return false;
    }
}

                                  
// Probe the center of the bed to determine its height in steps, taking probe offsets into account
// Refreshes the following variables, AND SHOULD BE CALLED BEFORE READING THEM:
//	bed_height
// 	mm_probe_height_to_trigger
bool ComprehensiveDeltaStrategy::find_bed_center_height() {

    float pos[3];       // For tracking axis positions
    float mm_all;       // Number of mm to get from home position to <0, 0, 0>
    float mm_alpha;     // Number of mm to get from home position to <0, 0, probe_height>
    int s;              // Number of steps to get from probe_height to trigger
 
    // Home, and store reported bed height (which should be at least approximately accurate)
    zprobe->home();
    THEKERNEL->robot->get_axis_position(pos);
    mm_all = pos[2];
    mm_alpha = mm_all - zprobe->getProbeHeight();

    // Move to <0, 0, probe_height>
    zprobe->coordinated_move(NAN, NAN, -mm_alpha, zprobe->getFastFeedrate(), true);

    // Move to probe's x/y offsets
    zprobe->coordinated_move(probe_offset_x, probe_offset_y, NAN, zprobe->getFastFeedrate(), true);

    // Find steps to probe trigger
    if(!zprobe->run_probe(s, false)) {
        return false;
    }
    mm_probe_height_to_trigger = zprobe->zsteps_to_mm(s);
_printf("[BH] s=%d phtt=%1.3f\n", s, mm_probe_height_to_trigger);

    // Bed height is therefore:
    //     distance from homed position to probe_height
    //   + (steps from probe_height to probe trigger / z steps per mm)
    //   + probe_offset_z
    bed_height = mm_alpha + mm_probe_height_to_trigger + probe_offset_z;

    // Tell the machine about the new height
    // FIXME: Endstops.cpp might have a more direct method for doing this - if so, that should be used instead!

    // -- Construct command
    char cmd[18];       // Should be enough for "M665 Z1000.12345"
    snprintf(cmd, 17, "M665 Z%1.5f", bed_height);
    _printf("[BH] Setting bed height: ");
    _printf(cmd);
    _printf("\n");
    
    // -- Send command
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    THEKERNEL->conveyor->wait_for_empty_queue();

    return true;

    /*
    _printf(
        "find_bed_center_height(): %d steps (%f mm) from probe_height (%f) to probe_offset_z (%f)\n",
        s, (float)(s / Z_STEPS_PER_MM), probe_height, probe_offset_z);
    _printf(
        "find_bed_center_height(): Bed is %f mm tall.\n", bed_height);
    */
}

bool ComprehensiveDeltaStrategy::do_probe_at(int &steps, float x, float y) {
    // Move to location, corrected for probe offset (if any)
    zprobe->coordinated_move(x + probe_offset_x, y + probe_offset_y, NAN, zprobe->getFastFeedrate(), false);

    // Run the probe
    if(!zprobe->run_probe(steps)) {
        _printf("do_probe_at(steps, %1.3f, %1.3f) - run_probe() returned false, s=%d.\n", x + probe_offset_x, y + probe_offset_y, steps);
        return false;
    }

    // Return probe to original Z
    zprobe->return_probe(steps);

    if(steps < 100) {
        _printf("do_probe_at(): steps=%d - this is much too small - is probe_height high enough?\n", steps);
        return false;
    } else {
        return true;
    }
}

// Flush the serial buffer
void ComprehensiveDeltaStrategy::flush() {
    THEKERNEL->call_event(ON_IDLE);
}
