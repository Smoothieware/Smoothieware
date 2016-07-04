/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/ActuatorCoordinates.h"
#include "Endstops.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Config.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "StreamOutputPool.h"
#include "StepTicker.h"
#include "BaseSolution.h"
#include "SerialMessage.h"

#include <ctype.h>

#define ALPHA_AXIS 0
#define BETA_AXIS  1
#define GAMMA_AXIS 2
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")
#define corexy_homing_checksum                  CHECKSUM("corexy_homing")
#define delta_homing_checksum                   CHECKSUM("delta_homing")
#define rdelta_homing_checksum                  CHECKSUM("rdelta_homing")
#define scara_homing_checksum                   CHECKSUM("scara_homing")

#define alpha_min_endstop_checksum       CHECKSUM("alpha_min_endstop")
#define beta_min_endstop_checksum        CHECKSUM("beta_min_endstop")
#define gamma_min_endstop_checksum       CHECKSUM("gamma_min_endstop")

#define alpha_max_endstop_checksum       CHECKSUM("alpha_max_endstop")
#define beta_max_endstop_checksum        CHECKSUM("beta_max_endstop")
#define gamma_max_endstop_checksum       CHECKSUM("gamma_max_endstop")

#define alpha_trim_checksum              CHECKSUM("alpha_trim")
#define beta_trim_checksum               CHECKSUM("beta_trim")
#define gamma_trim_checksum              CHECKSUM("gamma_trim")

// these values are in steps and should be deprecated
#define alpha_fast_homing_rate_checksum  CHECKSUM("alpha_fast_homing_rate")
#define beta_fast_homing_rate_checksum   CHECKSUM("beta_fast_homing_rate")
#define gamma_fast_homing_rate_checksum  CHECKSUM("gamma_fast_homing_rate")

#define alpha_slow_homing_rate_checksum  CHECKSUM("alpha_slow_homing_rate")
#define beta_slow_homing_rate_checksum   CHECKSUM("beta_slow_homing_rate")
#define gamma_slow_homing_rate_checksum  CHECKSUM("gamma_slow_homing_rate")

#define alpha_homing_retract_checksum    CHECKSUM("alpha_homing_retract")
#define beta_homing_retract_checksum     CHECKSUM("beta_homing_retract")
#define gamma_homing_retract_checksum    CHECKSUM("gamma_homing_retract")

// same as above but in user friendly mm/s and mm
#define alpha_fast_homing_rate_mm_checksum  CHECKSUM("alpha_fast_homing_rate_mm_s")
#define beta_fast_homing_rate_mm_checksum   CHECKSUM("beta_fast_homing_rate_mm_s")
#define gamma_fast_homing_rate_mm_checksum  CHECKSUM("gamma_fast_homing_rate_mm_s")

#define alpha_slow_homing_rate_mm_checksum  CHECKSUM("alpha_slow_homing_rate_mm_s")
#define beta_slow_homing_rate_mm_checksum   CHECKSUM("beta_slow_homing_rate_mm_s")
#define gamma_slow_homing_rate_mm_checksum  CHECKSUM("gamma_slow_homing_rate_mm_s")

#define alpha_homing_retract_mm_checksum    CHECKSUM("alpha_homing_retract_mm")
#define beta_homing_retract_mm_checksum     CHECKSUM("beta_homing_retract_mm")
#define gamma_homing_retract_mm_checksum    CHECKSUM("gamma_homing_retract_mm")

#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")
#define endstop_debounce_ms_checksum     CHECKSUM("endstop_debounce_ms")

#define alpha_homing_direction_checksum  CHECKSUM("alpha_homing_direction")
#define beta_homing_direction_checksum   CHECKSUM("beta_homing_direction")
#define gamma_homing_direction_checksum  CHECKSUM("gamma_homing_direction")
#define home_to_max_checksum             CHECKSUM("home_to_max")
#define home_to_min_checksum             CHECKSUM("home_to_min")
#define alpha_min_checksum               CHECKSUM("alpha_min")
#define beta_min_checksum                CHECKSUM("beta_min")
#define gamma_min_checksum               CHECKSUM("gamma_min")

#define alpha_max_checksum               CHECKSUM("alpha_max")
#define beta_max_checksum                CHECKSUM("beta_max")
#define gamma_max_checksum               CHECKSUM("gamma_max")

#define alpha_limit_enable_checksum      CHECKSUM("alpha_limit_enable")
#define beta_limit_enable_checksum       CHECKSUM("beta_limit_enable")
#define gamma_limit_enable_checksum      CHECKSUM("gamma_limit_enable")

#define home_z_first_checksum            CHECKSUM("home_z_first")
#define homing_order_checksum            CHECKSUM("homing_order")
#define move_to_origin_checksum          CHECKSUM("move_to_origin_after_home")

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())


// Homing States
enum {
    MOVING_TO_ENDSTOP_FAST, // homing move
    MOVING_TO_ENDSTOP_SLOW, // homing move
    MOVING_BACK,            // homing move
    NOT_HOMING,
    BACK_OFF_HOME,
    MOVE_TO_ORIGIN,
    LIMIT_TRIGGERED
};

Endstops::Endstops()
{
    this->status = NOT_HOMING;
    home_offset[0] = home_offset[1] = home_offset[2] = 0.0F;
    debounce.fill(0);
}

void Endstops::on_module_loaded()
{
    // Do not do anything if not enabled
    if ( THEKERNEL->config->value( endstops_module_enable_checksum )->by_default(true)->as_bool() == false ) {
        delete this;
        return;
    }

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);

    // Settings
    this->load_config();

    THEKERNEL->slow_ticker->attach(1000, this, &Endstops::read_endstops);
}

// Get config
void Endstops::load_config()
{
    this->pins[0].from_string( THEKERNEL->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[1].from_string( THEKERNEL->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[2].from_string( THEKERNEL->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[3].from_string( THEKERNEL->config->value(alpha_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[4].from_string( THEKERNEL->config->value(beta_max_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[5].from_string( THEKERNEL->config->value(gamma_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();

    // These are the old ones in steps still here for backwards compatibility
    this->fast_rates[0] =  THEKERNEL->config->value(alpha_fast_homing_rate_checksum     )->by_default(4000 )->as_number() / STEPS_PER_MM(0);
    this->fast_rates[1] =  THEKERNEL->config->value(beta_fast_homing_rate_checksum      )->by_default(4000 )->as_number() / STEPS_PER_MM(1);
    this->fast_rates[2] =  THEKERNEL->config->value(gamma_fast_homing_rate_checksum     )->by_default(6400 )->as_number() / STEPS_PER_MM(2);
    this->slow_rates[0] =  THEKERNEL->config->value(alpha_slow_homing_rate_checksum     )->by_default(2000 )->as_number() / STEPS_PER_MM(0);
    this->slow_rates[1] =  THEKERNEL->config->value(beta_slow_homing_rate_checksum      )->by_default(2000 )->as_number() / STEPS_PER_MM(1);
    this->slow_rates[2] =  THEKERNEL->config->value(gamma_slow_homing_rate_checksum     )->by_default(3200 )->as_number() / STEPS_PER_MM(2);
    this->retract_mm[0] =  THEKERNEL->config->value(alpha_homing_retract_checksum       )->by_default(400  )->as_number() / STEPS_PER_MM(0);
    this->retract_mm[1] =  THEKERNEL->config->value(beta_homing_retract_checksum        )->by_default(400  )->as_number() / STEPS_PER_MM(1);
    this->retract_mm[2] =  THEKERNEL->config->value(gamma_homing_retract_checksum       )->by_default(1600 )->as_number() / STEPS_PER_MM(2);

    // newer mm based config values override the old ones, convert to steps/mm and steps, defaults to what was set in the older config settings above
    this->fast_rates[0] = THEKERNEL->config->value(alpha_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[0])->as_number();
    this->fast_rates[1] = THEKERNEL->config->value(beta_fast_homing_rate_mm_checksum  )->by_default(this->fast_rates[1])->as_number();
    this->fast_rates[2] = THEKERNEL->config->value(gamma_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[2])->as_number();
    this->slow_rates[0] = THEKERNEL->config->value(alpha_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[0])->as_number();
    this->slow_rates[1] = THEKERNEL->config->value(beta_slow_homing_rate_mm_checksum  )->by_default(this->slow_rates[1])->as_number();
    this->slow_rates[2] = THEKERNEL->config->value(gamma_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[2])->as_number();
    this->retract_mm[0] = THEKERNEL->config->value(alpha_homing_retract_mm_checksum   )->by_default(this->retract_mm[0])->as_number();
    this->retract_mm[1] = THEKERNEL->config->value(beta_homing_retract_mm_checksum    )->by_default(this->retract_mm[1])->as_number();
    this->retract_mm[2] = THEKERNEL->config->value(gamma_homing_retract_mm_checksum   )->by_default(this->retract_mm[2])->as_number();

    // NOTE the debouce count is in milliseconds so probably does not need to beset anymore
    this->debounce_ms     = THEKERNEL->config->value(endstop_debounce_ms_checksum       )->by_default(0)->as_number();
    this->debounce_count  = THEKERNEL->config->value(endstop_debounce_count_checksum    )->by_default(100)->as_number();

    // get homing direction and convert to boolean where true is home to min, and false is home to max
    int home_dir                    = get_checksum(THEKERNEL->config->value(alpha_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[0]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(beta_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[1]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(gamma_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[2]         = home_dir != home_to_max_checksum;

    this->homing_position[0]        =  this->home_direction[0] ? THEKERNEL->config->value(alpha_min_checksum)->by_default(0)->as_number() : THEKERNEL->config->value(alpha_max_checksum)->by_default(200)->as_number();
    this->homing_position[1]        =  this->home_direction[1] ? THEKERNEL->config->value(beta_min_checksum )->by_default(0)->as_number() : THEKERNEL->config->value(beta_max_checksum )->by_default(200)->as_number();
    this->homing_position[2]        =  this->home_direction[2] ? THEKERNEL->config->value(gamma_min_checksum)->by_default(0)->as_number() : THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number();

    // used to set maximum movement on homing
    this->alpha_max= THEKERNEL->config->value(alpha_max_checksum)->by_default(500)->as_number();
    this->beta_max= THEKERNEL->config->value(beta_max_checksum)->by_default(500)->as_number();
    this->gamma_max= THEKERNEL->config->value(gamma_max_checksum)->by_default(500)->as_number();

    this->is_corexy                 =  THEKERNEL->config->value(corexy_homing_checksum)->by_default(false)->as_bool();
    this->is_delta                  =  THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_rdelta                 =  THEKERNEL->config->value(rdelta_homing_checksum)->by_default(false)->as_bool();
    this->is_scara                  =  THEKERNEL->config->value(scara_homing_checksum)->by_default(false)->as_bool();

    this->home_z_first              = THEKERNEL->config->value(home_z_first_checksum)->by_default(false)->as_bool();

    // see if an order has been specified, must be three characters, XYZ or YXZ etc
    string order = THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order = 0;
    if(order.size() == 3 && !(this->is_delta || this->is_rdelta)) {
        int shift = 0;
        for(auto c : order) {
            uint8_t i = toupper(c) - 'X';
            if(i > 2) { // bad value
                this->homing_order = 0;
                break;
            }
            homing_order |= (i << shift);
            shift += 2;
        }
    }

    // endstop trim used by deltas to do soft adjusting
    // on a delta homing to max, a negative trim value will move the carriage down, and a positive will move it up
    this->trim_mm[0] = THEKERNEL->config->value(alpha_trim_checksum )->by_default(0  )->as_number();
    this->trim_mm[1] = THEKERNEL->config->value(beta_trim_checksum  )->by_default(0  )->as_number();
    this->trim_mm[2] = THEKERNEL->config->value(gamma_trim_checksum )->by_default(0  )->as_number();

    // limits enabled
    this->limit_enable[X_AXIS] = THEKERNEL->config->value(alpha_limit_enable_checksum)->by_default(false)->as_bool();
    this->limit_enable[Y_AXIS] = THEKERNEL->config->value(beta_limit_enable_checksum)->by_default(false)->as_bool();
    this->limit_enable[Z_AXIS] = THEKERNEL->config->value(gamma_limit_enable_checksum)->by_default(false)->as_bool();

    // set to true by default for deltas due to trim, false on cartesians
    this->move_to_origin_after_home = THEKERNEL->config->value(move_to_origin_checksum)->by_default(is_delta)->as_bool();

    if(this->limit_enable[X_AXIS] || this->limit_enable[Y_AXIS] || this->limit_enable[Z_AXIS]) {
        register_for_event(ON_IDLE);
        if(this->is_delta || this->is_rdelta) {
            // we must enable all the limits not just one
            this->limit_enable[X_AXIS] = true;
            this->limit_enable[Y_AXIS] = true;
            this->limit_enable[Z_AXIS] = true;
        }
    }

    //
    if(this->is_delta || this->is_rdelta) {
        // some things must be the same or they will die, so force it here to avoid config errors
        this->fast_rates[1] = this->fast_rates[2] = this->fast_rates[0];
        this->slow_rates[1] = this->slow_rates[2] = this->slow_rates[0];
        this->retract_mm[1] = this->retract_mm[2] = this->retract_mm[0];
        this->home_direction[1] = this->home_direction[2] = this->home_direction[0];
        // NOTE homing_position for rdelta is the angle of the actuator not the cartesian position
        if(!this->is_rdelta) this->homing_position[0] = this->homing_position[1] = 0;
    }
}

bool Endstops::debounced_get(int pin)
{
    uint8_t debounce = 0;
    while(this->pins[pin].get()) {
        if ( ++debounce >= this->debounce_count ) {
            // pin triggered
            return true;
        }
    }
    return false;
}

static const char *endstop_names[] = {"min_x", "min_y", "min_z", "max_x", "max_y", "max_z"};

void Endstops::on_idle(void *argument)
{
    if(this->status == LIMIT_TRIGGERED) {
        // if we were in limit triggered see if it has been cleared
        for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if(this->limit_enable[c]) {
                std::array<int, 2> minmax{{0, 3}};
                // check min and max endstops
                for (int i : minmax) {
                    int n = c + i;
                    if(this->pins[n].get()) {
                        // still triggered, so exit
                        bounce_cnt = 0;
                        return;
                    }
                }
            }
        }
        if(++bounce_cnt > 10) { // can use less as it calls on_idle in between
            // clear the state
            this->status = NOT_HOMING;
        }
        return;

    } else if(this->status != NOT_HOMING) {
        // don't check while homing
        return;
    }

    for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if(this->limit_enable[c] && STEPPER[c]->is_moving()) {
            std::array<int, 2> minmax{{0, 3}};
            // check min and max endstops
            for (int i : minmax) {
                int n = c + i;
                if(debounced_get(n)) {
                    // endstop triggered
                    THEKERNEL->streams->printf("Limit switch %s was hit - reset or M999 required\n", endstop_names[n]);
                    this->status = LIMIT_TRIGGERED;
                    // disables heaters and motors, ignores incoming Gcode and flushes block queue
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    return;
                }
            }
        }
    }
}

// if limit switches are enabled, then we must move off of the endstop otherwise we won't be able to move
// checks if triggered and only backs off if triggered
void Endstops::back_off_home(std::bitset<3> axis)
{
    std::vector<std::pair<char, float>> params;
    this->status = BACK_OFF_HOME;

    // these are handled differently
    if(is_delta) {
        // Move off of the endstop using a regular relative move in Z only
        params.push_back({'Z', this->retract_mm[Z_AXIS] * (this->home_direction[Z_AXIS] ? 1 : -1)});

    } else {
        // cartesians, concatenate all the moves we need to do into one gcode
        for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if(!axis[c]) continue; // only for axes we asked to move

            // if not triggered no need to move off
            if(this->limit_enable[c] && debounced_get(c + (this->home_direction[c] ? 0 : 3)) ) {
                params.push_back({c + 'X', this->retract_mm[c] * (this->home_direction[c] ? 1 : -1)});
            }
        }
    }

    if(!params.empty()) {
        // Move off of the endstop using a regular relative move
        params.insert(params.begin(), {'G', 0});
        // use X slow rate to move, Z should have a max speed set anyway
        params.push_back({'F', this->slow_rates[X_AXIS] * 60.0F});
        char gcode_buf[64];
        append_parameters(gcode_buf, params, sizeof(gcode_buf));
        Gcode gc(gcode_buf, &(StreamOutput::NullStream));
        THEROBOT->push_state();
        THEROBOT->inch_mode = false;     // needs to be in mm
        THEROBOT->absolute_mode = false; // needs to be relative mode
        THEROBOT->on_gcode_received(&gc); // send to robot directly
        // Wait for above to finish
        THECONVEYOR->wait_for_idle();
        THEROBOT->pop_state();
    }

    this->status = NOT_HOMING;
}

// If enabled will move the head to 0,0 after homing, but only if X and Y were set to home
void Endstops::move_to_origin(std::bitset<3> axis)
{
    if(!is_delta && (!axis[X_AXIS] || !axis[Y_AXIS])) return; // ignore if X and Y not homing, unless delta

    // Do we need to check if we are already at 0,0? probably not as the G0 will not do anything if we are
    // float pos[3]; THEROBOT->get_axis_position(pos); if(pos[0] == 0 && pos[1] == 0) return;

    this->status = MOVE_TO_ORIGIN;
    // Move to center using a regular move, use slower of X and Y fast rate
    float rate = std::min(this->fast_rates[0], this->fast_rates[1]) * 60.0F;
    char buf[32];
    THEROBOT->push_state();
    THEROBOT->inch_mode = false;     // needs to be in mm
    snprintf(buf, sizeof(buf), "G53 G0 X0 Y0 F%1.4f", rate); // must use machine coordinates in case G92 or WCS is in effect
    struct SerialMessage message;
    message.message = buf;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); // as it is a multi G code command
    // Wait for above to finish
    THECONVEYOR->wait_for_idle();
    THEROBOT->pop_state();
    this->status = NOT_HOMING;
}

// Called every millisecond in an ISR
uint32_t Endstops::read_endstops(uint32_t dummy)
{
    if(this->status != MOVING_TO_ENDSTOP_SLOW && this->status != MOVING_TO_ENDSTOP_FAST) return 0; // not doing anything we need to monitor for

    if(!is_corexy) {
        // check each axis
        for ( int m = X_AXIS; m <= Z_AXIS; m++ ) {
            if(STEPPER[m]->is_moving()) {
                // if it is moving then we check the associated endstop, and debounce it
                if(this->pins[m + (this->home_direction[m] ? 0 : 3)].get()) {
                    if(debounce[m] < debounce_ms) {
                        debounce[m]++;
                    } else {
                        // we signal the motor to stop, which will preempt any moves on that axis
                        STEPPER[m]->stop_moving();
                    }

                } else {
                    // The endstop was not hit yet
                    debounce[m] = 0;
                }
            }
        }

    } else {
        // corexy is different as the actuators are not directly related to the XY axis
        // so we check the axis that is currently homing then stop all motors
        for ( int m = X_AXIS; m <= Z_AXIS; m++ ) {
            if(axis_to_home[m]) {
                if(this->pins[m + (this->home_direction[m] ? 0 : 3)].get()) {
                    if(debounce[m] < debounce_ms) {
                        debounce[m]++;
                    } else {
                        // we signal all the motors to stop, as on corexy X and Y motors will move for X and Y axis homing and we only hom eone axis at a time
                        STEPPER[X_AXIS]->stop_moving();
                        STEPPER[Y_AXIS]->stop_moving();
                        STEPPER[Z_AXIS]->stop_moving();
                    }

                } else {
                    // The endstop was not hit yet
                    debounce[m] = 0;
                }
            }
        }
    }

    return 0;
}

void Endstops::home_xy()
{
    if(axis_to_home[X_AXIS] && axis_to_home[Y_AXIS]) {
        // Home XY first so as not to slow them down by homing Z at the same time
        float delta[3] {alpha_max*2, beta_max*2, 0};
        if(this->home_direction[X_AXIS]) delta[X_AXIS]= -delta[X_AXIS];
        if(this->home_direction[Y_AXIS]) delta[Y_AXIS]= -delta[Y_AXIS];
        float feed_rate = std::min(fast_rates[X_AXIS], fast_rates[Y_AXIS]);
        THEROBOT->delta_move(delta, feed_rate, 3);

    } else if(axis_to_home[X_AXIS]) {
        // now home X only
        float delta[3] {alpha_max*2, 0, 0};
        if(this->home_direction[X_AXIS]) delta[X_AXIS]= -delta[X_AXIS];
        THEROBOT->delta_move(delta, fast_rates[X_AXIS], 3);

    } else if(axis_to_home[Y_AXIS]) {
        // now home Y only
        float delta[3] {0, beta_max*2, 0};
        if(this->home_direction[Y_AXIS]) delta[Y_AXIS]= -delta[Y_AXIS];
        THEROBOT->delta_move(delta, fast_rates[Y_AXIS], 3);
    }

    // Wait for axis to have homed
    THECONVEYOR->wait_for_idle();
}

void Endstops::home(std::bitset<3> a)
{
    // reset debounce counts
    debounce.fill(0);

    // turn off any compensation transform
    auto savect= THEROBOT->compensationTransform;
    THEROBOT->compensationTransform= nullptr;

    this->axis_to_home= a;

    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;

    THEROBOT->disable_segmentation= true; // we must disable segmentation as this won't work with it enabled

    if(!home_z_first) home_xy();

    if(axis_to_home[Z_AXIS]) {
        // now home z
        float delta[3] {0, 0, gamma_max*2}; // we go twice the maxz just in case it was set incorrectly
        if(this->home_direction[Z_AXIS]) delta[Z_AXIS]= -delta[Z_AXIS];
        THEROBOT->delta_move(delta, fast_rates[Z_AXIS], 3);
        // wait for Z
        THECONVEYOR->wait_for_idle();
    }

    if(home_z_first) home_xy();

    // TODO should check that the endstops were hit and it did not stop short for some reason
    // we did not complete movement the full distance if we hit the endstops
    THEROBOT->reset_position_from_current_actuator_position();

    // Move back a small distance for all homing axis
    this->status = MOVING_BACK;
    float delta[3]{0,0,0};
    // use minimum feed rate of all three axes that are being homed (sub optimal, but necessary)
    float feed_rate= slow_rates[X_AXIS];
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if(axis_to_home[c]) {
            delta[c]= this->retract_mm[c];
            if(!this->home_direction[c]) delta[c]= -delta[c];
            feed_rate= std::min(slow_rates[c], feed_rate);
        }
    }

    THEROBOT->delta_move(delta, feed_rate, 3);
    // wait until finished
    THECONVEYOR->wait_for_idle();

    // Start moving the axes towards the endstops slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if(axis_to_home[c]) {
            delta[c]= this->retract_mm[c]*2; // move further than we moved off to make sure we hit it cleanly
            if(this->home_direction[c]) delta[c]= -delta[c];
        }else{
            delta[c]= 0;
        }
    }
    THEROBOT->delta_move(delta, feed_rate, 3);
    // wait until finished
    THECONVEYOR->wait_for_idle();

    // TODO should check that the endstops were hit and it did not stop short for some reason
    // we did not complete movement the full distance if we hit the endstops
    THEROBOT->reset_position_from_current_actuator_position();

    THEROBOT->disable_segmentation= false;

    // restore compensationTransform
    THEROBOT->compensationTransform= savect;

    this->status = NOT_HOMING;
}

void Endstops::process_home_command(Gcode* gcode)
{
    if( (gcode->subcode == 0 && THEKERNEL->is_grbl_mode()) || (gcode->subcode == 2 && !THEKERNEL->is_grbl_mode()) ) {
        // G28 in grbl mode or G28.2 in normal mode will do a rapid to the predefined position
        // TODO spec says if XYZ specified move to them first then move to MCS of specifed axis
        char buf[32];
        snprintf(buf, sizeof(buf), "G53 G0 X%f Y%f", saved_position[X_AXIS], saved_position[Y_AXIS]); // must use machine coordinates in case G92 or WCS is in effect
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); // as it is a multi G code command
        return;

    } else if(THEKERNEL->is_grbl_mode() && gcode->subcode == 2) { // G28.2 in grbl mode forces homing (triggered by $H)
        // fall through so it does homing cycle

    } else if(gcode->subcode == 1) { // G28.1 set pre defined position
        // saves current position in absolute machine coordinates
        THEROBOT->get_axis_position(saved_position); // Only XY are used
        // Note the following is only meant to be used for recovering a saved position from config-override
        // Not a standard Gcode and not to be relied on
        if (gcode->has_letter('X')) saved_position[X_AXIS] = gcode->get_value('X');
        if (gcode->has_letter('Y')) saved_position[Y_AXIS] = gcode->get_value('Y');
        return;

    } else if(gcode->subcode == 3) { // G28.3 is a smoothie special it sets manual homing
        if(gcode->get_num_args() == 0) {
            THEROBOT->reset_axis_position(0, 0, 0);
        } else {
            // do a manual homing based on given coordinates, no endstops required
            if(gcode->has_letter('X')) THEROBOT->reset_axis_position(gcode->get_value('X'), X_AXIS);
            if(gcode->has_letter('Y')) THEROBOT->reset_axis_position(gcode->get_value('Y'), Y_AXIS);
            if(gcode->has_letter('Z')) THEROBOT->reset_axis_position(gcode->get_value('Z'), Z_AXIS);
        }
        return;

    } else if(gcode->subcode == 4) { // G28.4 is a smoothie special it sets manual homing based on the actuator position (used for rotary delta)
        // do a manual homing based on given coordinates, no endstops required
        ActuatorCoordinates ac;
        if(gcode->has_letter('X')) ac[0] =  gcode->get_value('X');
        if(gcode->has_letter('Y')) ac[1] =  gcode->get_value('Y');
        if(gcode->has_letter('Z')) ac[2] =  gcode->get_value('Z');
        THEROBOT->reset_actuator_position(ac);
        return;

    } else if(THEKERNEL->is_grbl_mode()) {
        gcode->stream->printf("error:Unsupported command\n");
        return;
    }

    // G28 is received, we have homing to do

    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // deltas, scaras always home Z axis only
    bool home_in_z = this->is_delta || this->is_rdelta || this->is_scara;

    // figure out which axis to home
    bitset<3> haxis;
    haxis.reset();

    if(!home_in_z) { // ie not a delta
        bool axis_speced = ( gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') );
        // only enable homing if the endstop is defined,
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if (this->pins[c + (this->home_direction[c] ? 0 : 3)].connected() && (!axis_speced || gcode->has_letter(c + 'X')) ) {
                haxis.set(c);
                // now reset axis to 0 as we do not know what state we are in
                THEROBOT->reset_axis_position(0, c);
            }
        }

    } else {
        // Only Z axis homes (even though all actuators move this is handled by arm solution)
        haxis.set(Z_AXIS);
    }

    // do the actual homing
    if(homing_order != 0) {
        // if an order has been specified do it in the specified order
        // homing order is 0b00ccbbaa where aa is 0,1,2 to specify the first axis, bb is the second and cc is the third
        // eg 0b00100001 would be Y X Z, 0b00100100 would be X Y Z
        for (uint8_t m = homing_order; m != 0; m >>= 2) {
            int a= (m & 0x03); // axis to home
            if(haxis[a]) { // if axis is selected to home
                std::bitset<3> bs;
                bs.set(a);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }

    } else if(is_corexy) {
        // corexy must home each axis individually
        for (int a = X_AXIS; a <= Z_AXIS; ++a) {
            if(haxis[a]) {
                std::bitset<3> bs;
                bs.set(a);
                home(bs);
            }
        }

    } else {
        // they could all home at the same time
        home(haxis);
    }

    // check if on_halt (eg kill)
    if(THEKERNEL->is_halted()) {
        if(!THEKERNEL->is_grbl_mode()) {
            THEKERNEL->streams->printf("Homing cycle aborted by kill\n");
        }
        return;
    }

    if(home_in_z) { // deltas only
        // Here's where we would have been if the endstops were perfectly trimmed
        // NOTE on a rotary delta home_offset is actuator position in degrees when homed and
        // home_offset is the theta offset for each actuator, so M206 is used to set theta offset for each actuator in degrees
        float ideal_position[3] = {
            this->homing_position[X_AXIS] + this->home_offset[X_AXIS],
            this->homing_position[Y_AXIS] + this->home_offset[Y_AXIS],
            this->homing_position[Z_AXIS] + this->home_offset[Z_AXIS]
        };

        bool has_endstop_trim = this->is_delta || this->is_scara;
        if (has_endstop_trim) {
            ActuatorCoordinates ideal_actuator_position;
            THEROBOT->arm_solution->cartesian_to_actuator(ideal_position, ideal_actuator_position);

            // We are actually not at the ideal position, but a trim away
            ActuatorCoordinates real_actuator_position = {
                ideal_actuator_position[X_AXIS] - this->trim_mm[X_AXIS],
                ideal_actuator_position[Y_AXIS] - this->trim_mm[Y_AXIS],
                ideal_actuator_position[Z_AXIS] - this->trim_mm[Z_AXIS]
            };

            float real_position[3];
            THEROBOT->arm_solution->actuator_to_cartesian(real_actuator_position, real_position);
            // Reset the actuator positions to correspond our real position
            THEROBOT->reset_axis_position(real_position[0], real_position[1], real_position[2]);

        } else {
            // without endstop trim, real_position == ideal_position
            if(is_rdelta) {
                // with a rotary delta we set the actuators angle then use the FK to calculate the resulting cartesian coordinates
                ActuatorCoordinates real_actuator_position = {ideal_position[0], ideal_position[1], ideal_position[2]};
                THEROBOT->reset_actuator_position(real_actuator_position);

            } else {
                // Reset the actuator positions to correspond our real position
                THEROBOT->reset_axis_position(ideal_position[0], ideal_position[1], ideal_position[2]);
            }
        }

    } else {
        // Zero the ax(i/e)s position, add in the home offset
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if (haxis[c]) { // if we requested this axis to home
                THEROBOT->reset_axis_position(this->homing_position[c] + this->home_offset[c], c);
            }
        }
    }

    // on some systems where 0,0 is bed center it is nice to have home goto 0,0 after homing
    // default is off for cartesian on for deltas
    if(!is_delta) {
        // NOTE a rotary delta usually has optical or hall-effect endstops so it is safe to go past them a little bit
        if(this->move_to_origin_after_home) move_to_origin(haxis);
        // if limit switches are enabled we must back off endstop after setting home
        back_off_home(haxis);

    } else if(this->move_to_origin_after_home || this->limit_enable[X_AXIS]) {
        // deltas are not left at 0,0 because of the trim settings, so move to 0,0 if requested, but we need to back off endstops first
        // also need to back off endstops if limits are enabled
        back_off_home(haxis);
        if(this->move_to_origin_after_home) move_to_origin(haxis);
    }
}

void Endstops::set_homing_offset(Gcode *gcode)
{
    // Similar to M206 and G92 but sets Homing offsets based on current position
    float cartesian[3];
    THEROBOT->get_axis_position(cartesian);    // get actual position from robot
    if (gcode->has_letter('X')) {
        home_offset[0] -= (cartesian[X_AXIS] - gcode->get_value('X'));
        THEROBOT->reset_axis_position(gcode->get_value('X'), X_AXIS);
    }
    if (gcode->has_letter('Y')) {
        home_offset[1] -= (cartesian[Y_AXIS] - gcode->get_value('Y'));
        THEROBOT->reset_axis_position(gcode->get_value('Y'), Y_AXIS);
    }
    if (gcode->has_letter('Z')) {
        home_offset[2] -= (cartesian[Z_AXIS] - gcode->get_value('Z'));
        THEROBOT->reset_axis_position(gcode->get_value('Z'), Z_AXIS);
    }

    gcode->stream->printf("Homing Offset: X %5.3f Y %5.3f Z %5.3f\n", home_offset[0], home_offset[1], home_offset[2]);
}

// Start homing sequences by response to GCode commands
void Endstops::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if ( gcode->has_g && gcode->g == 28) {
        process_home_command(gcode);

    } else if (gcode->has_m) {

        switch (gcode->m) {
            case 119: {
                for (int i = 0; i < 6; ++i) {
                    if(this->pins[i].connected())
                        gcode->stream->printf("%s:%d ", endstop_names[i], this->pins[i].get());
                }
                gcode->add_nl = true;

            }
            break;

            case 206: // M206 - set homing offset
                if(is_rdelta) return; // RotaryDeltaCalibration module will handle this

                if (gcode->has_letter('X')) home_offset[0] = gcode->get_value('X');
                if (gcode->has_letter('Y')) home_offset[1] = gcode->get_value('Y');
                if (gcode->has_letter('Z')) home_offset[2] = gcode->get_value('Z');
                gcode->stream->printf("X %5.3f Y %5.3f Z %5.3f\n", home_offset[0], home_offset[1], home_offset[2]);
                break;

            case 306: // set homing offset based on current position
                if(is_rdelta) return; // RotaryDeltaCalibration module will handle this

                set_homing_offset(gcode);
                break;

            case 500: // save settings
            case 503: // print settings
                if(!is_rdelta)
                    gcode->stream->printf(";Home offset (mm):\nM206 X%1.2f Y%1.2f Z%1.2f\n", home_offset[0], home_offset[1], home_offset[2]);
                else
                    gcode->stream->printf(";Theta offset (degrees):\nM206 A%1.5f B%1.5f C%1.5f\n", home_offset[0], home_offset[1], home_offset[2]);

                if (this->is_delta || this->is_scara) {
                    gcode->stream->printf(";Trim (mm):\nM666 X%1.3f Y%1.3f Z%1.3f\n", trim_mm[0], trim_mm[1], trim_mm[2]);
                    gcode->stream->printf(";Max Z\nM665 Z%1.3f\n", this->homing_position[2]);
                }
                if(saved_position[X_AXIS] != 0 || saved_position[Y_AXIS] != 0) {
                    gcode->stream->printf(";predefined position:\nG28.1 X%1.4f Y%1.4f\n", saved_position[X_AXIS], saved_position[Y_AXIS]);
                }
                break;

            case 665:
                if (this->is_delta || this->is_scara) { // M665 - set max gamma/z height
                    float gamma_max = this->homing_position[2];
                    if (gcode->has_letter('Z')) {
                        this->homing_position[2] = gamma_max = gcode->get_value('Z');
                    }
                    gcode->stream->printf("Max Z %8.3f ", gamma_max);
                    gcode->add_nl = true;
                }
                break;

            case 666:
                if(this->is_delta || this->is_scara) { // M666 - set trim for each axis in mm, NB negative mm trim is down
                    if (gcode->has_letter('X')) trim_mm[0] = gcode->get_value('X');
                    if (gcode->has_letter('Y')) trim_mm[1] = gcode->get_value('Y');
                    if (gcode->has_letter('Z')) trim_mm[2] = gcode->get_value('Z');

                    // print the current trim values in mm
                    gcode->stream->printf("X: %5.3f Y: %5.3f Z: %5.3f\n", trim_mm[0], trim_mm[1], trim_mm[2]);

                }
                break;

        }
    }
}

void Endstops::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(trim_checksum)) {
        pdr->set_data_ptr(&this->trim_mm);
        pdr->set_taken();

    } else if(pdr->second_element_is(home_offset_checksum)) {
        pdr->set_data_ptr(&this->home_offset);
        pdr->set_taken();

    } else if(pdr->second_element_is(saved_position_checksum)) {
        pdr->set_data_ptr(&this->saved_position);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_homing_status_checksum)) {
        bool *homing = static_cast<bool *>(pdr->get_data_ptr());
        *homing = this->status != NOT_HOMING;
        pdr->set_taken();
    }
}

void Endstops::on_set_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(trim_checksum)) {
        float *t = static_cast<float*>(pdr->get_data_ptr());
        this->trim_mm[0] = t[0];
        this->trim_mm[1] = t[1];
        this->trim_mm[2] = t[2];
        pdr->set_taken();

    } else if(pdr->second_element_is(home_offset_checksum)) {
        float *t = static_cast<float*>(pdr->get_data_ptr());
        if(!isnan(t[0])) this->home_offset[0] = t[0];
        if(!isnan(t[1])) this->home_offset[1] = t[1];
        if(!isnan(t[2])) this->home_offset[2] = t[2];
    }
}
