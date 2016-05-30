/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

#include "mbed.h" // for us_ticker_read()

#include <fastmath.h>
#include <string>
using std::string;

#include "Planner.h"
#include "Conveyor.h"
#include "Robot.h"
#include "nuts_bolts.h"
#include "Pin.h"
#include "StepperMotor.h"
#include "Gcode.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"
#include "arm_solutions/RotatableCartesianSolution.h"
#include "arm_solutions/LinearDeltaSolution.h"
#include "arm_solutions/RotaryDeltaSolution.h"
#include "arm_solutions/HBotSolution.h"
#include "arm_solutions/CoreXZSolution.h"
#include "arm_solutions/MorganSCARASolution.h"
#include "StepTicker.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"
#include "ExtruderPublicAccess.h"
#include "GcodeDispatch.h"


#define  default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define  default_feed_rate_checksum          CHECKSUM("default_feed_rate")
#define  mm_per_line_segment_checksum        CHECKSUM("mm_per_line_segment")
#define  delta_segments_per_second_checksum  CHECKSUM("delta_segments_per_second")
#define  mm_per_arc_segment_checksum         CHECKSUM("mm_per_arc_segment")
#define  arc_correction_checksum             CHECKSUM("arc_correction")
#define  x_axis_max_speed_checksum           CHECKSUM("x_axis_max_speed")
#define  y_axis_max_speed_checksum           CHECKSUM("y_axis_max_speed")
#define  z_axis_max_speed_checksum           CHECKSUM("z_axis_max_speed")
#define  segment_z_moves_checksum            CHECKSUM("segment_z_moves")
#define  save_g92_checksum                   CHECKSUM("save_g92")

// arm solutions
#define  arm_solution_checksum               CHECKSUM("arm_solution")
#define  cartesian_checksum                  CHECKSUM("cartesian")
#define  rotatable_cartesian_checksum        CHECKSUM("rotatable_cartesian")
#define  rostock_checksum                    CHECKSUM("rostock")
#define  linear_delta_checksum               CHECKSUM("linear_delta")
#define  rotary_delta_checksum               CHECKSUM("rotary_delta")
#define  delta_checksum                      CHECKSUM("delta")
#define  hbot_checksum                       CHECKSUM("hbot")
#define  corexy_checksum                     CHECKSUM("corexy")
#define  corexz_checksum                     CHECKSUM("corexz")
#define  kossel_checksum                     CHECKSUM("kossel")
#define  morgan_checksum                     CHECKSUM("morgan")

// new-style actuator stuff
#define  actuator_checksum                   CHEKCSUM("actuator")

#define  step_pin_checksum                   CHECKSUM("step_pin")
#define  dir_pin_checksum                    CHEKCSUM("dir_pin")
#define  en_pin_checksum                     CHECKSUM("en_pin")

#define  steps_per_mm_checksum               CHECKSUM("steps_per_mm")
#define  max_rate_checksum                   CHECKSUM("max_rate")

#define  alpha_checksum                      CHECKSUM("alpha")
#define  beta_checksum                       CHECKSUM("beta")
#define  gamma_checksum                      CHECKSUM("gamma")

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2

#define MOTION_MODE_SEEK 0 // G0
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2 // G2
#define MOTION_MODE_CCW_ARC 3 // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS 2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)

// The Robot converts GCodes into actual movements, and then adds them to the Planner, which passes them to the Conveyor so they can be added to the queue
// It takes care of cutting arcs into segments, same thing for line that are too long

Robot::Robot()
{
    this->inch_mode = false;
    this->absolute_mode = true;
    this->motion_mode =  MOTION_MODE_SEEK;
    this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    clear_vector(this->last_milestone);
    clear_vector(this->last_machine_position);
    this->arm_solution = NULL;
    seconds_per_minute = 60.0F;
    this->clearToolOffset();
    this->compensationTransform = nullptr;
    this->wcs_offsets.fill(wcs_t(0.0F, 0.0F, 0.0F));
    this->g92_offset = wcs_t(0.0F, 0.0F, 0.0F);
    this->next_command_is_MCS = false;
    this->disable_segmentation= false;
}

//Called when the module has just been loaded
void Robot::on_module_loaded()
{
    this->register_for_event(ON_GCODE_RECEIVED);

    // Configuration
    this->load_config();
}

#define ACTUATOR_CHECKSUMS(X) {     \
    CHECKSUM(X "_step_pin"),        \
    CHECKSUM(X "_dir_pin"),         \
    CHECKSUM(X "_en_pin"),          \
    CHECKSUM(X "_steps_per_mm"),    \
    CHECKSUM(X "_max_rate")         \
}

void Robot::load_config()
{
    // Arm solutions are used to convert positions in millimeters into position in steps for each stepper motor.
    // While for a cartesian arm solution, this is a simple multiplication, in other, less simple cases, there is some serious math to be done.
    // To make adding those solution easier, they have their own, separate object.
    // Here we read the config to find out which arm solution to use
    if (this->arm_solution) delete this->arm_solution;
    int solution_checksum = get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());
    // Note checksums are not const expressions when in debug mode, so don't use switch
    if(solution_checksum == hbot_checksum || solution_checksum == corexy_checksum) {
        this->arm_solution = new HBotSolution(THEKERNEL->config);

    } else if(solution_checksum == corexz_checksum) {
        this->arm_solution = new CoreXZSolution(THEKERNEL->config);

    } else if(solution_checksum == rostock_checksum || solution_checksum == kossel_checksum || solution_checksum == delta_checksum || solution_checksum ==  linear_delta_checksum) {
        this->arm_solution = new LinearDeltaSolution(THEKERNEL->config);

    } else if(solution_checksum == rotatable_cartesian_checksum) {
        this->arm_solution = new RotatableCartesianSolution(THEKERNEL->config);

    } else if(solution_checksum == rotary_delta_checksum) {
        this->arm_solution = new RotaryDeltaSolution(THEKERNEL->config);

    } else if(solution_checksum == morgan_checksum) {
        this->arm_solution = new MorganSCARASolution(THEKERNEL->config);

    } else if(solution_checksum == cartesian_checksum) {
        this->arm_solution = new CartesianSolution(THEKERNEL->config);

    } else {
        this->arm_solution = new CartesianSolution(THEKERNEL->config);
    }

    this->feed_rate           = THEKERNEL->config->value(default_feed_rate_checksum   )->by_default(  100.0F)->as_number();
    this->seek_rate           = THEKERNEL->config->value(default_seek_rate_checksum   )->by_default(  100.0F)->as_number();
    this->mm_per_line_segment = THEKERNEL->config->value(mm_per_line_segment_checksum )->by_default(    0.0F)->as_number();
    this->delta_segments_per_second = THEKERNEL->config->value(delta_segments_per_second_checksum )->by_default(0.0f   )->as_number();
    this->mm_per_arc_segment  = THEKERNEL->config->value(mm_per_arc_segment_checksum  )->by_default(    0.5f)->as_number();
    this->arc_correction      = THEKERNEL->config->value(arc_correction_checksum      )->by_default(    5   )->as_number();

    this->max_speeds[X_AXIS]  = THEKERNEL->config->value(x_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Y_AXIS]  = THEKERNEL->config->value(y_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Z_AXIS]  = THEKERNEL->config->value(z_axis_max_speed_checksum    )->by_default(  300.0F)->as_number() / 60.0F;

    this->segment_z_moves     = THEKERNEL->config->value(segment_z_moves_checksum     )->by_default(true)->as_bool();
    this->save_g92            = THEKERNEL->config->value(save_g92_checksum            )->by_default(false)->as_bool();

    // Make our 3 StepperMotors
    uint16_t const checksums[][5] = {
        ACTUATOR_CHECKSUMS("alpha"),
        ACTUATOR_CHECKSUMS("beta"),
        ACTUATOR_CHECKSUMS("gamma"),
#if MAX_ROBOT_ACTUATORS > 3
        ACTUATOR_CHECKSUMS("delta"),
        ACTUATOR_CHECKSUMS("epsilon"),
        ACTUATOR_CHECKSUMS("zeta")
#endif
    };
    constexpr size_t actuator_checksum_count = sizeof(checksums) / sizeof(checksums[0]);
    static_assert(actuator_checksum_count >= k_max_actuators, "Robot checksum array too small for k_max_actuators");

    size_t motor_count = std::min(this->arm_solution->get_actuator_count(), k_max_actuators);
    for (size_t a = 0; a < motor_count; a++) {
        Pin pins[3]; //step, dir, enable
        for (size_t i = 0; i < 3; i++) {
            pins[i].from_string(THEKERNEL->config->value(checksums[a][i])->by_default("nc")->as_string())->as_output();
        }
        actuators[a] = new StepperMotor(pins[0], pins[1], pins[2]);

        actuators[a]->change_steps_per_mm(THEKERNEL->config->value(checksums[a][3])->by_default(a == 2 ? 2560.0F : 80.0F)->as_number());
        actuators[a]->set_max_rate(THEKERNEL->config->value(checksums[a][4])->by_default(30000.0F)->as_number());
    }

    check_max_actuator_speeds(); // check the configs are sane

    // initialise actuator positions to current cartesian position (X0 Y0 Z0)
    // so the first move can be correct if homing is not performed
    ActuatorCoordinates actuator_pos;
    arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);
    for (size_t i = 0; i < actuators.size(); i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);

    //this->clearToolOffset();
}

void  Robot::push_state()
{
    bool am = this->absolute_mode;
    bool im = this->inch_mode;
    saved_state_t s(this->feed_rate, this->seek_rate, am, im, current_wcs);
    state_stack.push(s);
}

void Robot::pop_state()
{
    if(!state_stack.empty()) {
        auto s = state_stack.top();
        state_stack.pop();
        this->feed_rate = std::get<0>(s);
        this->seek_rate = std::get<1>(s);
        this->absolute_mode = std::get<2>(s);
        this->inch_mode = std::get<3>(s);
        this->current_wcs = std::get<4>(s);
    }
}

std::vector<Robot::wcs_t> Robot::get_wcs_state() const
{
    std::vector<wcs_t> v;
    v.push_back(wcs_t(current_wcs, MAX_WCS, 0));
    for(auto& i : wcs_offsets) {
        v.push_back(i);
    }
    v.push_back(g92_offset);
    v.push_back(tool_offset);
    return v;
}

int Robot::print_position(uint8_t subcode, char *buf, size_t bufsize) const
{
    // M114.1 is a new way to do this (similar to how GRBL does it).
    // it returns the realtime position based on the current step position of the actuators.
    // this does require a FK to get a machine position from the actuator position
    // and then invert all the transforms to get a workspace position from machine position
    // M114 just does it the old way uses last_milestone and does inversse transforms to get the requested position
    int n = 0;
    if(subcode == 0) { // M114 print WCS
        wcs_t pos= mcs2wcs(last_milestone);
        n = snprintf(buf, bufsize, "C: X:%1.4f Y:%1.4f Z:%1.4f", from_millimeters(std::get<X_AXIS>(pos)), from_millimeters(std::get<Y_AXIS>(pos)), from_millimeters(std::get<Z_AXIS>(pos)));

    } else if(subcode == 4) { // M114.4 print last milestone (which should be the same as machine position if axis are not moving and no level compensation)
        n = snprintf(buf, bufsize, "LMS: X:%1.4f Y:%1.4f Z:%1.4f", last_milestone[X_AXIS], last_milestone[Y_AXIS], last_milestone[Z_AXIS]);

    } else if(subcode == 5) { // M114.5 print last machine position (which should be the same as M114.1 if axis are not moving and no level compensation)
        n = snprintf(buf, bufsize, "LMP: X:%1.4f Y:%1.4f Z:%1.4f", last_machine_position[X_AXIS], last_machine_position[Y_AXIS], last_machine_position[Z_AXIS]);

    } else {
        // get real time positions
        // current actuator position in mm
        ActuatorCoordinates current_position{
            actuators[X_AXIS]->get_current_position(),
            actuators[Y_AXIS]->get_current_position(),
            actuators[Z_AXIS]->get_current_position()
        };

        // get machine position from the actuator position using FK
        float mpos[3];
        arm_solution->actuator_to_cartesian(current_position, mpos);

        if(subcode == 1) { // M114.1 print realtime WCS
            // FIXME this currently includes the compensation transform which is incorrect so will be slightly off if it is in effect (but by very little)
            wcs_t pos= mcs2wcs(mpos);
            n = snprintf(buf, bufsize, "C: X:%1.4f Y:%1.4f Z:%1.4f", from_millimeters(std::get<X_AXIS>(pos)), from_millimeters(std::get<Y_AXIS>(pos)), from_millimeters(std::get<Z_AXIS>(pos)));

        } else if(subcode == 2) { // M114.2 print realtime Machine coordinate system
            n = snprintf(buf, bufsize, "MPOS: X:%1.4f Y:%1.4f Z:%1.4f", mpos[X_AXIS], mpos[Y_AXIS], mpos[Z_AXIS]);

        } else if(subcode == 3) { // M114.3 print realtime actuator position
            n = snprintf(buf, bufsize, "APOS: A:%1.4f B:%1.4f C:%1.4f", current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
        }
    }
    return n;
}

// converts current last milestone (machine position without compensation transform) to work coordinate system (inverse transform)
Robot::wcs_t Robot::mcs2wcs(const Robot::wcs_t& pos) const
{
    return std::make_tuple(
        std::get<X_AXIS>(pos) - std::get<X_AXIS>(wcs_offsets[current_wcs]) + std::get<X_AXIS>(g92_offset) - std::get<X_AXIS>(tool_offset),
        std::get<Y_AXIS>(pos) - std::get<Y_AXIS>(wcs_offsets[current_wcs]) + std::get<Y_AXIS>(g92_offset) - std::get<Y_AXIS>(tool_offset),
        std::get<Z_AXIS>(pos) - std::get<Z_AXIS>(wcs_offsets[current_wcs]) + std::get<Z_AXIS>(g92_offset) - std::get<Z_AXIS>(tool_offset)
    );
}

// this does a sanity check that actuator speeds do not exceed steps rate capability
// we will override the actuator max_rate if the combination of max_rate and steps/sec exceeds base_stepping_frequency
void Robot::check_max_actuator_speeds()
{
    for (size_t i = 0; i < actuators.size(); i++) {
        float step_freq = actuators[i]->get_max_rate() * actuators[i]->get_steps_per_mm();
        if (step_freq > THEKERNEL->base_stepping_frequency) {
            actuators[i]->set_max_rate(floorf(THEKERNEL->base_stepping_frequency / actuators[i]->get_steps_per_mm()));
            THEKERNEL->streams->printf("WARNING: actuator %c rate exceeds base_stepping_frequency * alpha_steps_per_mm: %f, setting to %f\n", 'A' + i, step_freq, actuators[i]->max_rate);
        }
    }
}

//A GCode has been received
//See if the current Gcode line has some orders for us
void Robot::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    this->motion_mode = -1;

    if( gcode->has_g) {
        switch( gcode->g ) {
            case 0:  this->motion_mode = MOTION_MODE_SEEK;    break;
            case 1:  this->motion_mode = MOTION_MODE_LINEAR;  break;
            case 2:  this->motion_mode = MOTION_MODE_CW_ARC;  break;
            case 3:  this->motion_mode = MOTION_MODE_CCW_ARC; break;
            case 4: { // G4 pause
                uint32_t delay_ms = 0;
                if (gcode->has_letter('P')) {
                    delay_ms = gcode->get_int('P');
                }
                if (gcode->has_letter('S')) {
                    delay_ms += gcode->get_int('S') * 1000;
                }
                if (delay_ms > 0) {
                    // drain queue
                    THEKERNEL->conveyor->wait_for_empty_queue();
                    // wait for specified time
                    uint32_t start = us_ticker_read(); // mbed call
                    while ((us_ticker_read() - start) < delay_ms * 1000) {
                        THEKERNEL->call_event(ON_IDLE, this);
                        if(THEKERNEL->is_halted()) return;
                    }
                }
            }
            break;

            case 10: // G10 L2 [L20] Pn Xn Yn Zn set WCS
                if(gcode->has_letter('L') && (gcode->get_int('L') == 2 || gcode->get_int('L') == 20) && gcode->has_letter('P')) {
                    size_t n = gcode->get_uint('P');
                    if(n == 0) n = current_wcs; // set current coordinate system
                    else --n;
                    if(n < MAX_WCS) {
                        float x, y, z;
                        std::tie(x, y, z) = wcs_offsets[n];
                        if(gcode->get_int('L') == 20) {
                            // this makes the current machine position (less compensation transform) the offset
                            // get current position in WCS
                            wcs_t pos= mcs2wcs(last_milestone);

                            if(gcode->has_letter('X')){
                                x -= to_millimeters(gcode->get_value('X')) - std::get<X_AXIS>(pos);
                            }

                            if(gcode->has_letter('Y')){
                                y -= to_millimeters(gcode->get_value('Y')) - std::get<Y_AXIS>(pos);
                            }
                            if(gcode->has_letter('Z')) {
                                z -= to_millimeters(gcode->get_value('Z')) - std::get<Z_AXIS>(pos);
                            }

                        } else {
                            // the value is the offset from machine zero
                            if(gcode->has_letter('X')) x = to_millimeters(gcode->get_value('X'));
                            if(gcode->has_letter('Y')) y = to_millimeters(gcode->get_value('Y'));
                            if(gcode->has_letter('Z')) z = to_millimeters(gcode->get_value('Z'));
                        }
                        wcs_offsets[n] = wcs_t(x, y, z);
                    }
                }
                break;

            case 17: this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);   break;
            case 18: this->select_plane(X_AXIS, Z_AXIS, Y_AXIS);   break;
            case 19: this->select_plane(Y_AXIS, Z_AXIS, X_AXIS);   break;
            case 20: this->inch_mode = true;   break;
            case 21: this->inch_mode = false;   break;

            case 54: case 55: case 56: case 57: case 58: case 59:
                // select WCS 0-8: G54..G59, G59.1, G59.2, G59.3
                current_wcs = gcode->g - 54;
                if(gcode->g == 59 && gcode->subcode > 0) {
                    current_wcs += gcode->subcode;
                    if(current_wcs >= MAX_WCS) current_wcs = MAX_WCS - 1;
                }
                break;

            case 90: this->absolute_mode = true;   break;
            case 91: this->absolute_mode = false;   break;

            case 92: {
                if(gcode->subcode == 1 || gcode->subcode == 2 || gcode->get_num_args() == 0) {
                    // reset G92 offsets to 0
                    g92_offset = wcs_t(0, 0, 0);

                } else if(gcode->subcode == 3) {
                    // initialize G92 to the specified values, only used for saving it with M500
                    float x= 0, y= 0, z= 0;
                    if(gcode->has_letter('X')) x= gcode->get_value('X');
                    if(gcode->has_letter('Y')) y= gcode->get_value('Y');
                    if(gcode->has_letter('Z')) z= gcode->get_value('Z');
                    g92_offset = wcs_t(x, y, z);

                } else {
                    // standard setting of the g92 offsets, making current WCS position whatever the coordinate arguments are
                    float x, y, z;
                    std::tie(x, y, z) = g92_offset;
                    // get current position in WCS
                    wcs_t pos= mcs2wcs(last_milestone);

                    // adjust g92 offset to make the current wpos == the value requested
                    if(gcode->has_letter('X')){
                        x += to_millimeters(gcode->get_value('X')) - std::get<X_AXIS>(pos);
                    }
                    if(gcode->has_letter('Y')){
                        y += to_millimeters(gcode->get_value('Y')) - std::get<Y_AXIS>(pos);
                    }
                    if(gcode->has_letter('Z')) {
                        z += to_millimeters(gcode->get_value('Z')) - std::get<Z_AXIS>(pos);
                    }
                    g92_offset = wcs_t(x, y, z);
                }

                return;
            }
        }

    } else if( gcode->has_m) {
        switch( gcode->m ) {
            // case 0: // M0 feed hold, (M0.1 is release feed hold, except we are in feed hold)
            //     if(THEKERNEL->is_grbl_mode()) THEKERNEL->set_feed_hold(gcode->subcode == 0);
            //     break;

            case 30: // M30 end of program in grbl mode (otherwise it is delete sdcard file)
                if(!THEKERNEL->is_grbl_mode()) break;
                // fall through to M2
            case 2: // M2 end of program
                current_wcs = 0;
                absolute_mode = true;
                break;

            case 92: // M92 - set steps per mm
                if (gcode->has_letter('X'))
                    actuators[0]->change_steps_per_mm(this->to_millimeters(gcode->get_value('X')));
                if (gcode->has_letter('Y'))
                    actuators[1]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Y')));
                if (gcode->has_letter('Z'))
                    actuators[2]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Z')));

                gcode->stream->printf("X:%f Y:%f Z:%f ", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm);
                gcode->add_nl = true;
                check_max_actuator_speeds();
                return;

            case 114:{
                char buf[64];
                int n= print_position(gcode->subcode, buf, sizeof buf);
                if(n > 0) gcode->txt_after_ok.append(buf, n);
                return;
            }

            case 120: // push state
                push_state();
                break;

            case 121: // pop state
                pop_state();
                break;

            case 203: // M203 Set maximum feedrates in mm/sec
                if (gcode->has_letter('X'))
                    this->max_speeds[X_AXIS] = gcode->get_value('X');
                if (gcode->has_letter('Y'))
                    this->max_speeds[Y_AXIS] = gcode->get_value('Y');
                if (gcode->has_letter('Z'))
                    this->max_speeds[Z_AXIS] = gcode->get_value('Z');
                for (size_t i = 0; i < 3 && i < actuators.size(); i++) {
                    if (gcode->has_letter('A' + i))
                        actuators[i]->set_max_rate(gcode->get_value('A' + i));
                }
                check_max_actuator_speeds();

                if(gcode->get_num_args() == 0) {
                    gcode->stream->printf("X:%g Y:%g Z:%g",
                                          this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS]);
                    for (size_t i = 0; i < 3 && i < actuators.size(); i++) {
                        gcode->stream->printf(" %c : %g", 'A' + i, actuators[i]->get_max_rate()); //xxx
                    }
                    gcode->add_nl = true;
                }
                break;

            case 204: // M204 Snnn - set acceleration to nnn, Znnn sets z acceleration
                if (gcode->has_letter('S')) {
                    float acc = gcode->get_value('S'); // mm/s^2
                    // enforce minimum
                    if (acc < 1.0F)
                        acc = 1.0F;
                    THEKERNEL->planner->acceleration = acc;
                }
                if (gcode->has_letter('Z')) {
                    float acc = gcode->get_value('Z'); // mm/s^2
                    // enforce positive
                    if (acc < 0.0F)
                        acc = 0.0F;
                    THEKERNEL->planner->z_acceleration = acc;
                }
                break;

            case 205: // M205 Xnnn - set junction deviation, Z - set Z junction deviation, Snnn - Set minimum planner speed, Ynnn - set minimum step rate
                if (gcode->has_letter('X')) {
                    float jd = gcode->get_value('X');
                    // enforce minimum
                    if (jd < 0.0F)
                        jd = 0.0F;
                    THEKERNEL->planner->junction_deviation = jd;
                }
                if (gcode->has_letter('Z')) {
                    float jd = gcode->get_value('Z');
                    // enforce minimum, -1 disables it and uses regular junction deviation
                    if (jd < -1.0F)
                        jd = -1.0F;
                    THEKERNEL->planner->z_junction_deviation = jd;
                }
                if (gcode->has_letter('S')) {
                    float mps = gcode->get_value('S');
                    // enforce minimum
                    if (mps < 0.0F)
                        mps = 0.0F;
                    THEKERNEL->planner->minimum_planner_speed = mps;
                }
                if (gcode->has_letter('Y')) {
                    actuators[0]->default_minimum_actuator_rate = gcode->get_value('Y');
                }
                break;

            case 220: // M220 - speed override percentage
                if (gcode->has_letter('S')) {
                    float factor = gcode->get_value('S');
                    // enforce minimum 10% speed
                    if (factor < 10.0F)
                        factor = 10.0F;
                    // enforce maximum 10x speed
                    if (factor > 1000.0F)
                        factor = 1000.0F;

                    seconds_per_minute = 6000.0F / factor;
                } else {
                    gcode->stream->printf("Speed factor at %6.2f %%\n", 6000.0F / seconds_per_minute);
                }
                break;

            case 400: // wait until all moves are done up to this point
                THEKERNEL->conveyor->wait_for_empty_queue();
                break;

            case 500: // M500 saves some volatile settings to config override file
            case 503: { // M503 just prints the settings
                gcode->stream->printf(";Steps per unit:\nM92 X%1.5f Y%1.5f Z%1.5f\n", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm);
                gcode->stream->printf(";Acceleration mm/sec^2:\nM204 S%1.5f Z%1.5f\n", THEKERNEL->planner->acceleration, THEKERNEL->planner->z_acceleration);
                gcode->stream->printf(";X- Junction Deviation, Z- Z junction deviation, S - Minimum Planner speed mm/sec:\nM205 X%1.5f Z%1.5f S%1.5f\n", THEKERNEL->planner->junction_deviation, THEKERNEL->planner->z_junction_deviation, THEKERNEL->planner->minimum_planner_speed);
                gcode->stream->printf(";Max feedrates in mm/sec, XYZ cartesian, ABC actuator:\nM203 X%1.5f Y%1.5f Z%1.5f",
                                      this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS]);
                for (size_t i = 0; i < 3 && i < actuators.size(); i++) {
                    gcode->stream->printf(" %c%1.5f", 'A' + i, actuators[i]->get_max_rate());
                }
                gcode->stream->printf("\n");

                // get or save any arm solution specific optional values
                BaseSolution::arm_options_t options;
                if(arm_solution->get_optional(options) && !options.empty()) {
                    gcode->stream->printf(";Optional arm solution specific settings:\nM665");
                    for(auto &i : options) {
                        gcode->stream->printf(" %c%1.4f", i.first, i.second);
                    }
                    gcode->stream->printf("\n");
                }

                // save wcs_offsets and current_wcs
                // TODO this may need to be done whenever they change to be compliant
                gcode->stream->printf(";WCS settings\n");
                gcode->stream->printf("%s\n", wcs2gcode(current_wcs).c_str());
                int n = 1;
                for(auto &i : wcs_offsets) {
                    if(i != wcs_t(0, 0, 0)) {
                        float x, y, z;
                        std::tie(x, y, z) = i;
                        gcode->stream->printf("G10 L2 P%d X%f Y%f Z%f ; %s\n", n, x, y, z, wcs2gcode(n-1).c_str());
                    }
                    ++n;
                }
                if(save_g92) {
                    // linuxcnc saves G92, so we do too if configured, default is to not save to maintain backward compatibility
                    // also it needs to be used to set Z0 on rotary deltas as M206/306 can't be used, so saving it is necessary in that case
                    if(g92_offset != wcs_t(0, 0, 0)) {
                        float x, y, z;
                        std::tie(x, y, z) = g92_offset;
                        gcode->stream->printf("G92.3 X%f Y%f Z%f\n", x, y, z); // sets G92 to the specified values
                    }
                }
            }
            break;

            case 665: { // M665 set optional arm solution variables based on arm solution.
                // the parameter args could be any letter each arm solution only accepts certain ones
                BaseSolution::arm_options_t options = gcode->get_args();
                options.erase('S'); // don't include the S
                options.erase('U'); // don't include the U
                if(options.size() > 0) {
                    // set the specified options
                    arm_solution->set_optional(options);
                }
                options.clear();
                if(arm_solution->get_optional(options)) {
                    // foreach optional value
                    for(auto &i : options) {
                        // print all current values of supported options
                        gcode->stream->printf("%c: %8.4f ", i.first, i.second);
                        gcode->add_nl = true;
                    }
                }

                if(gcode->has_letter('S')) { // set delta segments per second, not saved by M500
                    this->delta_segments_per_second = gcode->get_value('S');
                    gcode->stream->printf("Delta segments set to %8.4f segs/sec\n", this->delta_segments_per_second);

                } else if(gcode->has_letter('U')) { // or set mm_per_line_segment, not saved by M500
                    this->mm_per_line_segment = gcode->get_value('U');
                    this->delta_segments_per_second = 0;
                    gcode->stream->printf("mm per line segment set to %8.4f\n", this->mm_per_line_segment);
                }

                break;
            }
        }
    }

    if( this->motion_mode >= 0) {
        process_move(gcode);
    }

    next_command_is_MCS = false; // must be on same line as G0 or G1
}

// process a G0/G1/G2/G3
void Robot::process_move(Gcode *gcode)
{
    // we have a G0/G1/G2/G3 so extract parameters and apply offsets to get machine coordinate target
    float param[3]{NAN, NAN, NAN};
    for(int i= X_AXIS; i <= Z_AXIS; ++i) {
        char letter= 'X'+i;
        if( gcode->has_letter(letter) ) {
            param[i] = this->to_millimeters(gcode->get_value(letter));
        }
    }

    float offset[3]{0,0,0};
    for(char letter = 'I'; letter <= 'K'; letter++) {
        if( gcode->has_letter(letter) ) {
            offset[letter - 'I'] = this->to_millimeters(gcode->get_value(letter));
        }
    }

    // calculate target in machine coordinates (less compensation transform which needs to be done after segmentation)
    float target[3]{last_milestone[X_AXIS], last_milestone[Y_AXIS], last_milestone[Z_AXIS]};
    if(!next_command_is_MCS) {
        if(this->absolute_mode) {
            // apply wcs offsets and g92 offset and tool offset
            if(!isnan(param[X_AXIS])) {
                target[X_AXIS]= param[X_AXIS] + std::get<X_AXIS>(wcs_offsets[current_wcs]) - std::get<X_AXIS>(g92_offset) + std::get<X_AXIS>(tool_offset);
            }

            if(!isnan(param[Y_AXIS])) {
                target[Y_AXIS]= param[Y_AXIS] + std::get<Y_AXIS>(wcs_offsets[current_wcs]) - std::get<Y_AXIS>(g92_offset) + std::get<Y_AXIS>(tool_offset);
            }

            if(!isnan(param[Z_AXIS])) {
                target[Z_AXIS]= param[Z_AXIS] + std::get<Z_AXIS>(wcs_offsets[current_wcs]) - std::get<Z_AXIS>(g92_offset) + std::get<Z_AXIS>(tool_offset);
            }

        }else{
            // they are deltas from the last_milestone if specified
            for(int i= X_AXIS; i <= Z_AXIS; ++i) {
                if(!isnan(param[i])) target[i] = param[i] + last_milestone[i];
            }
        }

    }else{
        // already in machine coordinates, we do not add tool offset for that
        for(int i= X_AXIS; i <= Z_AXIS; ++i) {
            if(!isnan(param[i])) target[i] = param[i];
        }
    }

    if( gcode->has_letter('F') ) {
        if( this->motion_mode == MOTION_MODE_SEEK )
            this->seek_rate = this->to_millimeters( gcode->get_value('F') );
        else
            this->feed_rate = this->to_millimeters( gcode->get_value('F') );
    }

    bool moved= false;
    //Perform any physical actions
    switch(this->motion_mode) {
        case MOTION_MODE_CANCEL:
            break;
        case MOTION_MODE_SEEK:
            moved= this->append_line(gcode, target, this->seek_rate / seconds_per_minute );
            break;
        case MOTION_MODE_LINEAR:
            moved= this->append_line(gcode, target, this->feed_rate / seconds_per_minute );
            break;
        case MOTION_MODE_CW_ARC:
        case MOTION_MODE_CCW_ARC:
            moved= this->compute_arc(gcode, offset, target );
            break;
    }

    if(moved) {
        // set last_milestone to the calculated target
        memcpy(this->last_milestone, target, sizeof(this->last_milestone));
    }
}

// We received a new gcode, and one of the functions
// determined the distance for that given gcode. So now we can attach this gcode to the right block
// and continue
void Robot::distance_in_gcode_is_known(Gcode * gcode)
{
    //If the queue is empty, execute immediately, otherwise attach to the last added block
    THEKERNEL->conveyor->append_gcode(gcode);
}

// reset the machine position for all axis. Used for homing.
// During homing compensation is turned off (actually not used as it drives steppers directly)
// once homed and reset_axis called compensation is used for the move to origin and back off home if enabled,
// so in those cases the final position is compensated.
void Robot::reset_axis_position(float x, float y, float z)
{
    // these are set to the same as compensation was not used to get to the current position
    last_machine_position[X_AXIS]= last_milestone[X_AXIS] = x;
    last_machine_position[Y_AXIS]= last_milestone[Y_AXIS] = y;
    last_machine_position[Z_AXIS]= last_milestone[Z_AXIS] = z;

    // now set the actuator positions to match
    ActuatorCoordinates actuator_pos;
    arm_solution->cartesian_to_actuator(this->last_machine_position, actuator_pos);
    for (size_t i = 0; i < actuators.size(); i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);
}

// Reset the position for an axis (used in homing)
void Robot::reset_axis_position(float position, int axis)
{
    last_milestone[axis] = position;
    reset_axis_position(last_milestone[X_AXIS], last_milestone[Y_AXIS], last_milestone[Z_AXIS]);
}

// similar to reset_axis_position but directly sets the actuator positions in actuators units (eg mm for cartesian, degrees for rotary delta)
// then sets the axis positions to match. currently only called from Endstops.cpp
void Robot::reset_actuator_position(const ActuatorCoordinates &ac)
{
    for (size_t i = 0; i < actuators.size(); i++)
        actuators[i]->change_last_milestone(ac[i]);

    // now correct axis positions then recorrect actuator to account for rounding
    reset_position_from_current_actuator_position();
}

// Use FK to find out where actuator is and reset to match
void Robot::reset_position_from_current_actuator_position()
{
    ActuatorCoordinates actuator_pos;
    for (size_t i = 0; i < actuators.size(); i++) {
        // NOTE actuator::current_position is curently NOT the same as actuator::last_milestone after an abrupt abort
        actuator_pos[i] = actuators[i]->get_current_position();
    }

    // discover machine position from where actuators actually are
    arm_solution->actuator_to_cartesian(actuator_pos, last_machine_position);
    // FIXME problem is this includes any compensation transform, and without an inverse compensation we cannot get a correct last_milestone
    memcpy(last_milestone, last_machine_position, sizeof last_milestone);

    // now reset actuator::last_milestone, NOTE this may lose a little precision as FK is not always entirely accurate.
    // NOTE This is required to sync the machine position with the actuator position, we do a somewhat redundant cartesian_to_actuator() call
    // to get everything in perfect sync.
    arm_solution->cartesian_to_actuator(last_machine_position, actuator_pos);
    for (size_t i = 0; i < actuators.size(); i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);
}

// Convert target (in machine coordinates) from millimeters to steps, and append this to the planner
// target is in machine coordinates without the compensation transform, however we save a last_machine_position that includes
// all transforms and is what we actually convert to actuator positions
bool Robot::append_milestone(Gcode * gcode, const float target[], float rate_mm_s)
{
    float deltas[3];
    float unit_vec[3];
    ActuatorCoordinates actuator_pos;
    float transformed_target[3]; // adjust target for bed compensation and WCS offsets
    float millimeters_of_travel;

    // catch negative or zero feed rates and return the same error as GRBL does
    if(rate_mm_s <= 0.0F) {
        gcode->is_error= true;
        gcode->txt_after_ok= (rate_mm_s == 0 ? "Undefined feed rate" : "feed rate < 0");
        return false;
    }

    // unity transform by default
    memcpy(transformed_target, target, sizeof(transformed_target));

    // check function pointer and call if set to transform the target to compensate for bed
    if(compensationTransform) {
        // some compensation strategies can transform XYZ, some just change Z
        compensationTransform(transformed_target);
    }

    // find distance moved by each axis, use transformed target from the current machine position
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++) {
        deltas[axis] = transformed_target[axis] - last_machine_position[axis];
    }

    // Compute how long this move moves, so we can attach it to the block for later use
    millimeters_of_travel = sqrtf( powf( deltas[X_AXIS], 2 ) +  powf( deltas[Y_AXIS], 2 ) +  powf( deltas[Z_AXIS], 2 ) );

    // it is unlikely but we need to protect against divide by zero, so ignore insanely small moves here
    // as the last milestone won't be updated we do not actually lose any moves as they will be accounted for in the next move
    if(millimeters_of_travel < 0.00001F) return false;

    // this is the machine position
    memcpy(this->last_machine_position, transformed_target, sizeof(this->last_machine_position));

    // find distance unit vector
    for (int i = 0; i < 3; i++)
        unit_vec[i] = deltas[i] / millimeters_of_travel;

    // Do not move faster than the configured cartesian limits
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++) {
        if ( max_speeds[axis] > 0 ) {
            float axis_speed = fabs(unit_vec[axis] * rate_mm_s);

            if (axis_speed > max_speeds[axis])
                rate_mm_s *= ( max_speeds[axis] / axis_speed );
        }
    }

    // find actuator position given the machine position, use actual adjusted target
    arm_solution->cartesian_to_actuator( this->last_machine_position, actuator_pos );

    float isecs = rate_mm_s / millimeters_of_travel;
    // check per-actuator speed limits
    for (size_t actuator = 0; actuator < actuators.size(); actuator++) {
        float actuator_rate  = fabsf(actuator_pos[actuator] - actuators[actuator]->last_milestone_mm) * isecs;
        if (actuator_rate > actuators[actuator]->get_max_rate()) {
            rate_mm_s *= (actuators[actuator]->get_max_rate() / actuator_rate);
            isecs = rate_mm_s / millimeters_of_travel;
        }
    }

    // Append the block to the planner
    THEKERNEL->planner->append_block( actuator_pos, rate_mm_s, millimeters_of_travel, unit_vec );

    return true;
}

// Append a move to the queue ( cutting it into segments if needed )
bool Robot::append_line(Gcode *gcode, const float target[], float rate_mm_s )
{
    // Find out the distance for this move in MCS
    // NOTE we need to do sqrt here as this setting of millimeters_of_travel is used by extruder and other modules even if there is no XYZ move
    gcode->millimeters_of_travel = sqrtf(powf( target[X_AXIS] - last_milestone[X_AXIS], 2 ) +  powf( target[Y_AXIS] - last_milestone[Y_AXIS], 2 ) +  powf( target[Z_AXIS] - last_milestone[Z_AXIS], 2 ));

    // We ignore non- XYZ moves ( for example, extruder moves are not XYZ moves )
    if( gcode->millimeters_of_travel < 0.00001F ) return false;

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // if we have volumetric limits enabled we calculate the volume for this move and limit the rate if it exceeds the stated limit
    // Note we need to be using volumetric extrusion for this to work as Ennn is in mm³ not mm
    // We also check we are not exceeding the E max_speed for the current extruder
    // We ask Extruder to do all the work, but as Extruder won't even see this gcode until after it has been planned
    // we need to ask it now passing in the relevant data.
    // NOTE we need to do this before we segment the line (for deltas)
    if(gcode->has_letter('E')) {
        float data[2];
        data[0] = gcode->get_value('E'); // E target (may be absolute or relative)
        data[1] = rate_mm_s / gcode->millimeters_of_travel; // inverted seconds for the move
        if(PublicData::set_value(extruder_checksum, target_checksum, data)) {
            rate_mm_s *= data[1];
            //THEKERNEL->streams->printf("Extruder has changed the rate by %f to %f\n", data[1], rate_mm_s);
        }
    }

    // We cut the line into smaller segments. This is only needed on a cartesian robot for zgrid, but always necessary for robots with rotational axes like Deltas.
    // In delta robots either mm_per_line_segment can be used OR delta_segments_per_second
    // The latter is more efficient and avoids splitting fast long lines into very small segments, like initial z move to 0, it is what Johanns Marlin delta port does
    uint16_t segments;

    if(this->disable_segmentation || (!segment_z_moves && !gcode->has_letter('X') && !gcode->has_letter('Y'))) {
        segments= 1;

    } else if(this->delta_segments_per_second > 1.0F) {
        // enabled if set to something > 1, it is set to 0.0 by default
        // segment based on current speed and requested segments per second
        // the faster the travel speed the fewer segments needed
        // NOTE rate is mm/sec and we take into account any speed override
        float seconds = gcode->millimeters_of_travel / rate_mm_s;
        segments = max(1.0F, ceilf(this->delta_segments_per_second * seconds));
        // TODO if we are only moving in Z on a delta we don't really need to segment at all

    } else {
        if(this->mm_per_line_segment == 0.0F) {
            segments = 1; // don't split it up
        } else {
            segments = ceilf( gcode->millimeters_of_travel / this->mm_per_line_segment);
        }
    }

    bool moved= false;
    if (segments > 1) {
        // A vector to keep track of the endpoint of each segment
        float segment_delta[3];
        float segment_end[3]{last_milestone[X_AXIS], last_milestone[Y_AXIS], last_milestone[Z_AXIS]};

        // How far do we move each segment?
        for (int i = X_AXIS; i <= Z_AXIS; i++)
            segment_delta[i] = (target[i] - last_milestone[i]) / segments;

        // segment 0 is already done - it's the end point of the previous move so we start at segment 1
        // We always add another point after this loop so we stop at segments-1, ie i < segments
        for (int i = 1; i < segments; i++) {
            if(THEKERNEL->is_halted()) return false; // don't queue any more segments
            for(int axis = X_AXIS; axis <= Z_AXIS; axis++ )
                segment_end[axis] += segment_delta[axis];

            // Append the end of this segment to the queue
            bool b= this->append_milestone(gcode, segment_end, rate_mm_s);
            moved= moved || b;
        }
    }

    // Append the end of this full move to the queue
    if(this->append_milestone(gcode, target, rate_mm_s)) moved= true;

    this->next_command_is_MCS = false; // always reset this

    if(moved) {
        // if adding these blocks didn't start executing, do that now
        THEKERNEL->conveyor->ensure_running();
    }

    return moved;
}


// Append an arc to the queue ( cutting it into segments as needed )
bool Robot::append_arc(Gcode * gcode, const float target[], const float offset[], float radius, bool is_clockwise )
{

    // Scary math
    float center_axis0 = this->last_milestone[this->plane_axis_0] + offset[this->plane_axis_0];
    float center_axis1 = this->last_milestone[this->plane_axis_1] + offset[this->plane_axis_1];
    float linear_travel = target[this->plane_axis_2] - this->last_milestone[this->plane_axis_2];
    float r_axis0 = -offset[this->plane_axis_0]; // Radius vector from center to current location
    float r_axis1 = -offset[this->plane_axis_1];
    float rt_axis0 = target[this->plane_axis_0] - center_axis0;
    float rt_axis1 = target[this->plane_axis_1] - center_axis1;

    // Patch from GRBL Firmware - Christoph Baumann 04072015
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2f(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if (is_clockwise) { // Correct atan2 output per direction
        if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= (2 * (float)M_PI); }
    } else {
        if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += (2 * (float)M_PI); }
    }

    // Find the distance for this gcode
    gcode->millimeters_of_travel = hypotf(angular_travel * radius, fabsf(linear_travel));

    // We don't care about non-XYZ moves ( for example the extruder produces some of those )
    if( gcode->millimeters_of_travel < 0.00001F ) {
        return false;
    }

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // Figure out how many segments for this gcode
    uint16_t segments = floorf(gcode->millimeters_of_travel / this->mm_per_arc_segment);

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = linear_travel / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi) cos(phi] * r ;
    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all float numbers are single precision on the Arduino. (True float precision will not have
    round off issues for CNC applications.) Single precision error can accumulate to be greater than
    tool precision in some cases. Therefore, arc path correction is implemented.

    Small angle approximation may be used to reduce computation overhead further. This approximation
    holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
    theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
    to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
    numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
    issue for CNC machines with the single precision Arduino calculations.
    This approximation also allows mc_arc to immediately insert a line segment into the planner
    without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
    a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
    This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5F * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[3];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[this->plane_axis_2] = this->last_milestone[this->plane_axis_2];

    bool moved= false;
    for (i = 1; i < segments; i++) { // Increment (segments-1)
        if(THEKERNEL->is_halted()) return false; // don't queue any more segments

        if (count < this->arc_correction ) {
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cosf(i * theta_per_segment);
            sin_Ti = sinf(i * theta_per_segment);
            r_axis0 = -offset[this->plane_axis_0] * cos_Ti + offset[this->plane_axis_1] * sin_Ti;
            r_axis1 = -offset[this->plane_axis_0] * sin_Ti - offset[this->plane_axis_1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[this->plane_axis_0] = center_axis0 + r_axis0;
        arc_target[this->plane_axis_1] = center_axis1 + r_axis1;
        arc_target[this->plane_axis_2] += linear_per_segment;

        // Append this segment to the queue
        bool b= this->append_milestone(gcode, arc_target, this->feed_rate / seconds_per_minute);
        moved= moved || b;
    }

    // Ensure last segment arrives at target location.
    if(this->append_milestone(gcode, target, this->feed_rate / seconds_per_minute)) moved= true;

    return moved;
}

// Do the math for an arc and add it to the queue
bool Robot::compute_arc(Gcode * gcode, const float offset[], const float target[])
{

    // Find the radius
    float radius = hypotf(offset[this->plane_axis_0], offset[this->plane_axis_1]);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( this->motion_mode == MOTION_MODE_CW_ARC ) {
        is_clockwise = true;
    }

    // Append arc
    return this->append_arc(gcode, target, offset,  radius, is_clockwise );
}


float Robot::theta(float x, float y)
{
    float t = atanf(x / fabs(y));
    if (y > 0) {
        return(t);
    } else {
        if (t > 0) {
            return(M_PI - t);
        } else {
            return(-M_PI - t);
        }
    }
}

void Robot::select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2)
{
    this->plane_axis_0 = axis_0;
    this->plane_axis_1 = axis_1;
    this->plane_axis_2 = axis_2;
}

void Robot::clearToolOffset()
{
    this->tool_offset= wcs_t(0,0,0);
}

void Robot::setToolOffset(const float offset[3])
{
    this->tool_offset= wcs_t(offset[0], offset[1], offset[2]);
}

float Robot::get_feed_rate() const
{
    return THEKERNEL->gcode_dispatch->get_modal_command() == 0 ? seek_rate : feed_rate;
}
