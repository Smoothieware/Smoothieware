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
#include "Endstops.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Stepper.h"
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
#include "Pauser.h"
#include "StepTicker.h"

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

#define homing_order_checksum            CHECKSUM("homing_order")
#define move_to_origin_checksum          CHECKSUM("move_to_origin_after_home")

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())


// Homing States
enum{
    MOVING_TO_ENDSTOP_FAST, // homing move
    MOVING_BACK,            // homing move
    MOVING_TO_ENDSTOP_SLOW, // homing move
    NOT_HOMING,
    BACK_OFF_HOME,
    MOVE_TO_ORIGIN,
    LIMIT_TRIGGERED
};

Endstops::Endstops()
{
    this->status = NOT_HOMING;
    home_offset[0] = home_offset[1] = home_offset[2] = 0.0F;
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

    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){acceleration_tick(); });

    // Settings
    this->on_config_reload(this);
}

// Get config
void Endstops::on_config_reload(void *argument)
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

    this->is_corexy                 =  THEKERNEL->config->value(corexy_homing_checksum)->by_default(false)->as_bool();
    this->is_delta                  =  THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_scara                  =  THEKERNEL->config->value(scara_homing_checksum)->by_default(false)->as_bool();

    // see if an order has been specified, must be three characters, XYZ or YXZ etc
    string order= THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order= 0;
    if(order.size() == 3 && !this->is_delta) {
        int shift= 0;
        for(auto c : order) {
            uint8_t i= toupper(c) - 'X';
            if(i > 2) { // bad value
                this->homing_order= 0;
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
    this->limit_enable[X_AXIS]= THEKERNEL->config->value(alpha_limit_enable_checksum)->by_default(false)->as_bool();
    this->limit_enable[Y_AXIS]= THEKERNEL->config->value(beta_limit_enable_checksum)->by_default(false)->as_bool();
    this->limit_enable[Z_AXIS]= THEKERNEL->config->value(gamma_limit_enable_checksum)->by_default(false)->as_bool();

    this->move_to_origin_after_home= THEKERNEL->config->value(move_to_origin_checksum)->by_default(false)->as_bool();

    if(this->limit_enable[X_AXIS] || this->limit_enable[Y_AXIS] || this->limit_enable[Z_AXIS]){
        register_for_event(ON_IDLE);
    }
}

static const char *endstop_names[]= {"min_x", "min_y", "min_z", "max_x", "max_y", "max_z"};

void Endstops::on_idle(void *argument)
{
    if(this->status == LIMIT_TRIGGERED) {
        // if we were in limit triggered see if it has been cleared
        for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if(this->limit_enable[c]) {
                std::array<int, 2> minmax{{0, 3}};
                // check min and max endstops
                for (int i : minmax) {
                    int n= c+i;
                    if(this->pins[n].get()) {
                        // still triggered, so exit
                        bounce_cnt= 0;
                        return;
                    }
                }
            }
        }
        if(++bounce_cnt > 10) { // can use less as it calls on_idle in between
            // clear the state
            this->status= NOT_HOMING;
        }
        return;

    }else if(this->status != NOT_HOMING) {
        // don't check while homing
        return;
    }

    for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if(this->limit_enable[c] && STEPPER[c]->is_moving()) {
            std::array<int, 2> minmax{{0, 3}};
            // check min and max endstops
            for (int i : minmax) {
                int n= c+i;
                uint8_t debounce= 0;
                while(this->pins[n].get()) {
                    if ( ++debounce >= debounce_count ) {
                        // endstop triggered
                        THEKERNEL->streams->printf("Limit switch %s was hit - reset or M999 required\n", endstop_names[n]);
                        this->status= LIMIT_TRIGGERED;
                        // disables heaters and motors, ignores incoming Gcode and flushes block queue
                        THEKERNEL->call_event(ON_HALT, nullptr);
                        return;
                    }
                }
            }
        }
    }
}

// if limit switches are enabled, then we must move off of the endstop otherwise we won't be able to move
// checks if triggered and only backs off if triggered
void Endstops::back_off_home(char axes_to_move)
{
    this->status = BACK_OFF_HOME;
    for( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if( ((axes_to_move >> c ) & 1) == 0) continue; // only for axes we asked to move
        if(this->limit_enable[c]) {
            if( !this->pins[c + (this->home_direction[c] ? 0 : 3)].get() ) continue; // if not triggered no need to move off

            // Move off of the endstop using a regular relative move
            char buf[32];
            snprintf(buf, sizeof(buf), "G0 %c%1.4f F%1.4f", c+'X', this->retract_mm[c]*(this->home_direction[c]?1:-1), this->fast_rates[c]*60.0F);
            Gcode gc(buf, &(StreamOutput::NullStream));
            bool oldmode= THEKERNEL->robot->absolute_mode;
            THEKERNEL->robot->absolute_mode= false; // needs to be relative mode
            THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
            THEKERNEL->robot->absolute_mode= oldmode; // restore mode
        }
    }
    // Wait for above to finish
    THEKERNEL->conveyor->wait_for_empty_queue();
    this->status = NOT_HOMING;
}

// If enabled will move the head to 0,0 after homing, but only if X and Y were set to home
void Endstops::move_to_origin(char axes_to_move)
{
    if( (axes_to_move&0x03) != 3 ) return; // ignore if X and Y not homing

    // Do we need to check if we are already at 0,0? probably not as the G0 will not do anything if we are
    // float pos[3]; THEKERNEL->robot->get_axis_position(pos); if(pos[0] == 0 && pos[1] == 0) return;

    this->status = MOVE_TO_ORIGIN;
    // Move to center using a regular move, use slower of X and Y fast rate
    float rate= min(this->fast_rates[0], this->fast_rates[1])*60.0F;
    char buf[32];
    snprintf(buf, sizeof(buf), "G0 X0 Y0 F%1.4f", rate);
    Gcode gc(buf, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly

    // Wait for above to finish
    THEKERNEL->conveyor->wait_for_empty_queue();
    this->status = NOT_HOMING;
}

void Endstops::wait_for_homed(char axes_to_move)
{
    bool running = true;
    unsigned int debounce[3] = {0, 0, 0};
    while (running) {
        running = false;
        THEKERNEL->call_event(ON_IDLE);
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if ( ( axes_to_move >> c ) & 1 ) {
                if ( this->pins[c + (this->home_direction[c] ? 0 : 3)].get() ) {
                    if ( debounce[c] < debounce_count ) {
                        debounce[c]++;
                        running = true;
                    } else if ( STEPPER[c]->is_moving() ) {
                        STEPPER[c]->move(0, 0);
                        axes_to_move &= ~(1<<c); // no need to check it again
                    }
                } else {
                    // The endstop was not hit yet
                    running = true;
                    debounce[c] = 0;
                }
            }
        }
    }
}

void Endstops::do_homing_cartesian(char axes_to_move)
{
    // this homing works for cartesian and delta printers
    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if ( ( axes_to_move >> c) & 1 ) {
            this->feed_rate[c]= this->fast_rates[c];
            STEPPER[c]->move(this->home_direction[c], 10000000, 0);
        }
    }

    // Wait for all axes to have homed
    this->wait_for_homed(axes_to_move);

    // Move back a small distance
    this->status = MOVING_BACK;
    bool inverted_dir;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if ( ( axes_to_move >> c ) & 1 ) {
            inverted_dir = !this->home_direction[c];
            this->feed_rate[c]= this->slow_rates[c];
            STEPPER[c]->move(inverted_dir, this->retract_mm[c]*STEPS_PER_MM(c), 0);
        }
    }

    // Wait for moves to be done
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if (  ( axes_to_move >> c ) & 1 ) {
            while ( STEPPER[c]->is_moving() ) {
                THEKERNEL->call_event(ON_IDLE);
            }
        }
    }

    // Start moving the axes to the origin slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if ( ( axes_to_move >> c ) & 1 ) {
            this->feed_rate[c]= this->slow_rates[c];
            STEPPER[c]->move(this->home_direction[c], 10000000, 0);
        }
    }

    // Wait for all axes to have homed
    this->wait_for_homed(axes_to_move);

    if (this->is_delta || this->is_scara) {
        // move for soft trim
        this->status = MOVING_BACK;
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if ( this->trim_mm[c] != 0.0F && ( axes_to_move >> c ) & 1 ) {
                inverted_dir = this->home_direction[c];
                // move up or down depending on sign of trim, -ive is down away from home
                if (this->trim_mm[c] < 0) inverted_dir = !inverted_dir;
                this->feed_rate[c]= this->slow_rates[c];
                STEPPER[c]->move(inverted_dir, abs(round(this->trim_mm[c]*STEPS_PER_MM(c))), 0);
            }
        }

        // Wait for moves to be done
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if (  ( axes_to_move >> c ) & 1 ) {
                //THEKERNEL->streams->printf("axis %c \r\n", c );
                while ( STEPPER[c]->is_moving() ) {
                    THEKERNEL->call_event(ON_IDLE);
                }
            }
        }
    }

    // Homing is done
    this->status = NOT_HOMING;
}

void Endstops::wait_for_homed_corexy(int axis)
{
    bool running = true;
    unsigned int debounce[3] = {0, 0, 0};
    while (running) {
        running = false;
        THEKERNEL->call_event(ON_IDLE);
        if ( this->pins[axis + (this->home_direction[axis] ? 0 : 3)].get() ) {
            if ( debounce[axis] < debounce_count ) {
                debounce[axis] ++;
                running = true;
            } else {
                // turn both off if running
                if (STEPPER[X_AXIS]->is_moving()) STEPPER[X_AXIS]->move(0, 0);
                if (STEPPER[Y_AXIS]->is_moving()) STEPPER[Y_AXIS]->move(0, 0);
            }
        } else {
            // The endstop was not hit yet
            running = true;
            debounce[axis] = 0;
        }
    }
}

void Endstops::corexy_home(int home_axis, bool dirx, bool diry, float fast_rate, float slow_rate, unsigned int retract_steps)
{
    this->status = MOVING_TO_ENDSTOP_FAST;
    this->feed_rate[X_AXIS]= fast_rate;
    STEPPER[X_AXIS]->move(dirx, 10000000, 0);
    this->feed_rate[Y_AXIS]= fast_rate;
    STEPPER[Y_AXIS]->move(diry, 10000000, 0);

    // wait for primary axis
    this->wait_for_homed_corexy(home_axis);

    // Move back a small distance
    this->status = MOVING_BACK;
    this->feed_rate[X_AXIS]= slow_rate;
    STEPPER[X_AXIS]->move(!dirx, retract_steps, 0);
    this->feed_rate[Y_AXIS]= slow_rate;
    STEPPER[Y_AXIS]->move(!diry, retract_steps, 0);

    // wait until done
    while ( STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving()) {
        THEKERNEL->call_event(ON_IDLE);
    }

    // Start moving the axes to the origin slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    this->feed_rate[X_AXIS]= slow_rate;
    STEPPER[X_AXIS]->move(dirx, 10000000, 0);
    this->feed_rate[Y_AXIS]= slow_rate;
    STEPPER[Y_AXIS]->move(diry, 10000000, 0);

    // wait for primary axis
    this->wait_for_homed_corexy(home_axis);
}

// this homing works for HBots/CoreXY
void Endstops::do_homing_corexy(char axes_to_move)
{
    // TODO should really make order configurable, and select whether to allow XY to home at the same time, diagonally
    // To move XY at the same time only one motor needs to turn, determine which motor and which direction based on min or max directions
    // allow to move until an endstop triggers, then stop that motor. Speed up when moving diagonally to match X or Y speed
    // continue moving in the direction not yet triggered (which means two motors turning) until endstop hit

    if((axes_to_move & 0x03) == 0x03) { // both X and Y need Homing
        // determine which motor to turn and which way
        bool dirx= this->home_direction[X_AXIS];
        bool diry= this->home_direction[Y_AXIS];
        int motor;
        bool dir;
        if(dirx && diry) { // min/min
            motor= X_AXIS;
            dir= true;
        }else if(dirx && !diry) { // min/max
            motor= Y_AXIS;
            dir= true;
        }else if(!dirx && diry) { // max/min
            motor= Y_AXIS;
            dir= false;
        }else if(!dirx && !diry) { // max/max
            motor= X_AXIS;
            dir= false;
        }

        // then move both X and Y until one hits the endstop
        this->status = MOVING_TO_ENDSTOP_FAST;
         // need to allow for more ground covered when moving diagonally
        this->feed_rate[motor]= this->fast_rates[motor]*1.4142;
        STEPPER[motor]->move(dir, 10000000, 0);
        // wait until either X or Y hits the endstop
        bool running= true;
        while (running) {
            THEKERNEL->call_event(ON_IDLE);
            for(int m=X_AXIS;m<=Y_AXIS;m++) {
                if(this->pins[m + (this->home_direction[m] ? 0 : 3)].get()) {
                    // turn off motor
                    if(STEPPER[motor]->is_moving()) STEPPER[motor]->move(0, 0);
                    running= false;
                    break;
                }
            }
        }
    }

    // move individual axis
    if (axes_to_move & 0x01) { // Home X, which means both X and Y in same direction
        bool dir= this->home_direction[X_AXIS];
        corexy_home(X_AXIS, dir, dir, this->fast_rates[X_AXIS], this->slow_rates[X_AXIS], this->retract_mm[X_AXIS]*STEPS_PER_MM(X_AXIS));
    }

    if (axes_to_move & 0x02) { // Home Y, which means both X and Y in different directions
        bool dir= this->home_direction[Y_AXIS];
        corexy_home(Y_AXIS, dir, !dir, this->fast_rates[Y_AXIS], this->slow_rates[Y_AXIS], this->retract_mm[Y_AXIS]*STEPS_PER_MM(Y_AXIS));
    }

    if (axes_to_move & 0x04) { // move Z
        do_homing_cartesian(0x04); // just home normally for Z
    }

    // Homing is done
    this->status = NOT_HOMING;
}

void Endstops::home(char axes_to_move)
{
    // not a block move so disable the last tick setting
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        STEPPER[c]->set_moved_last_block(false);
    }

    if (is_corexy){
        // corexy/HBot homing
        do_homing_corexy(axes_to_move);
    }else{
        // cartesian/delta homing
        do_homing_cartesian(axes_to_move);
    }
}

// Start homing sequences by response to GCode commands
void Endstops::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if ( gcode->has_g) {
        if ( gcode->g == 28 ) {
            gcode->mark_as_taken();
            // G28 is received, we have homing to do

            // First wait for the queue to be empty
            THEKERNEL->conveyor->wait_for_empty_queue();

            // Do we move select axes or all of them
            char axes_to_move = 0;
            // only enable homing if the endstop is defined, deltas, scaras always home all axis
            bool home_all = this->is_delta || this->is_scara || !( gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') );

            for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
                if ( (home_all || gcode->has_letter(c+'X')) && this->pins[c + (this->home_direction[c] ? 0 : 3)].connected() ) {
                    axes_to_move += ( 1 << c );
                }
            }

            // Enable the motors
            THEKERNEL->stepper->turn_enable_pins_on();

            // do the actual homing
            if(homing_order != 0){
                // if an order has been specified do it in the specified order
                // homing order is 0b00ccbbaa where aa is 0,1,2 to specify the first axis, bb is the second and cc is the third
                // eg 0b00100001 would be Y X Z, 0b00100100 would be X Y Z
                for (uint8_t m = homing_order; m != 0; m >>= 2) {
                    int a= (1 << (m & 0x03)); // axis to move
                    if((a & axes_to_move) != 0)
                        home(a);
                }
            }else {
                // they all home at the same time
                home(axes_to_move);
            }

            if(home_all) {
                // for deltas this may be important rather than setting each individually
                THEKERNEL->robot->reset_axis_position(
                    this->homing_position[X_AXIS] + this->home_offset[X_AXIS],
                    this->homing_position[Y_AXIS] + this->home_offset[Y_AXIS],
                    this->homing_position[Z_AXIS] + this->home_offset[Z_AXIS]);
            }else{
                // Zero the ax(i/e)s position, add in the home offset
                for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
                    if ( (axes_to_move >> c)  & 1 ) {
                        THEKERNEL->robot->reset_axis_position(this->homing_position[c] + this->home_offset[c], c);
                    }
                }
            }

            // on some systems where 0,0 is bed center it is noce to have home goto 0,0 after homing
            // default is off
            if(this->move_to_origin_after_home)
                move_to_origin(axes_to_move);

            // if limit switches are enabled we must back off endstop after setting home
            back_off_home(axes_to_move);
        }

    } else if (gcode->has_m) {
        switch (gcode->m) {
            case 119: {
                for (int i = 0; i < 6; ++i) {
                    if(this->pins[i].connected())
                        gcode->stream->printf("%s:%d ", endstop_names[i], this->pins[i].get());
                }
                gcode->add_nl= true;
                gcode->mark_as_taken();
            }
            break;

            case 206: // M206 - set homing offset
                if (gcode->has_letter('X')) home_offset[0] = gcode->get_value('X');
                if (gcode->has_letter('Y')) home_offset[1] = gcode->get_value('Y');
                if (gcode->has_letter('Z')) home_offset[2] = gcode->get_value('Z');
                gcode->stream->printf("X %5.3f Y %5.3f Z %5.3f\n", home_offset[0], home_offset[1], home_offset[2]);
                gcode->mark_as_taken();
                break;

            case 306: // Similar to M206 and G92 but sets Homing offsets based on current position, Would be M207 but that is taken
                {
                    float cartesian[3];
                    THEKERNEL->robot->get_axis_position(cartesian);    // get actual position from robot
                    if (gcode->has_letter('X')){
                        home_offset[0] -= (cartesian[X_AXIS] - gcode->get_value('X'));
                        THEKERNEL->robot->reset_axis_position(gcode->get_value('X'), X_AXIS);
                    }
                    if (gcode->has_letter('Y')) {
                        home_offset[1] -= (cartesian[Y_AXIS] - gcode->get_value('Y'));
                        THEKERNEL->robot->reset_axis_position(gcode->get_value('Y'), Y_AXIS);
                    }
                    if (gcode->has_letter('Z')) {
                        home_offset[2] -= (cartesian[Z_AXIS] - gcode->get_value('Z'));
                        THEKERNEL->robot->reset_axis_position(gcode->get_value('Z'), Z_AXIS);
                    }

                    gcode->stream->printf("Homing Offset: X %5.3f Y %5.3f Z %5.3f\n", home_offset[0], home_offset[1], home_offset[2]);
                    gcode->mark_as_taken();
                }
                break;

            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Home offset (mm):\nM206 X%1.2f Y%1.2f Z%1.2f\n", home_offset[0], home_offset[1], home_offset[2]);
                if (this->is_delta || this->is_scara) {
                    gcode->stream->printf(";Trim (mm):\nM666 X%1.3f Y%1.3f Z%1.3f\n", trim_mm[0], trim_mm[1], trim_mm[2]);
                    gcode->stream->printf(";Max Z\nM665 Z%1.3f\n", this->homing_position[2]);
                }
                gcode->mark_as_taken();
                break;

            case 665: { // M665 - set max gamma/z height
                gcode->mark_as_taken();
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
                    gcode->mark_as_taken();
                }
            break;

            // NOTE this is to test accuracy of lead screws etc.
            case 910: { // M910 - move specific number of raw steps
                // Enable the motors
                THEKERNEL->stepper->turn_enable_pins_on();

                int x= 0, y=0 , z= 0, f= 200*16;
                if (gcode->has_letter('F')) f = gcode->get_value('F');
                if (gcode->has_letter('X')) {
                    x = gcode->get_value('X');
                    STEPPER[X_AXIS]->move(x<0, abs(x), f);
                }
                if (gcode->has_letter('Y')) {
                    y = gcode->get_value('Y');
                    STEPPER[Y_AXIS]->move(y<0, abs(y), f);
                }
                if (gcode->has_letter('Z')) {
                    z = gcode->get_value('Z');
                    STEPPER[Z_AXIS]->move(z<0, abs(z), f);
                }
                gcode->stream->printf("Moved X %d Y %d Z %d F %d steps\n", x, y, z, f);
                gcode->mark_as_taken();
                break;
            }
        }
    }
}

// Called periodically to change the speed to match acceleration
void Endstops::acceleration_tick(void)
{
    if(this->status >= NOT_HOMING) return; // nothing to do, only do this when moving for homing sequence

    // foreach stepper that is moving
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if( !STEPPER[c]->is_moving() ) continue;

        uint32_t current_rate = STEPPER[c]->get_steps_per_second();
        uint32_t target_rate = floorf(this->feed_rate[c]*STEPS_PER_MM(c));
        float acc= (c==Z_AXIS) ? THEKERNEL->planner->get_z_acceleration() : THEKERNEL->planner->get_acceleration();
        if( current_rate < target_rate ){
            uint32_t rate_increase = floorf((acc/THEKERNEL->acceleration_ticks_per_second)*STEPS_PER_MM(c));
            current_rate = min( target_rate, current_rate + rate_increase );
        }
        if( current_rate > target_rate ){ current_rate = target_rate; }

        // steps per second
        STEPPER[c]->set_speed(current_rate);
    }

    return;
}

void Endstops::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(trim_checksum)) {
        pdr->set_data_ptr(&this->trim_mm);
        pdr->set_taken();

    }else if(pdr->second_element_is(home_offset_checksum)) {
        pdr->set_data_ptr(&this->home_offset);
        pdr->set_taken();
    }
}

void Endstops::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(endstops_checksum)) return;

    if(pdr->second_element_is(trim_checksum)) {
        float *t= static_cast<float*>(pdr->get_data_ptr());
        this->trim_mm[0]= t[0];
        this->trim_mm[1]= t[1];
        this->trim_mm[2]= t[2];
        pdr->set_taken();

    }else if(pdr->second_element_is(home_offset_checksum)) {
        float *t= static_cast<float*>(pdr->get_data_ptr());
        if(!isnan(t[0])) this->home_offset[0]= t[0];
        if(!isnan(t[1])) this->home_offset[1]= t[1];
        if(!isnan(t[2])) this->home_offset[2]= t[2];
    }
}
