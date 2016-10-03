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

// OLD deprecated syntax
#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")

#define ENDSTOP_CHECKSUMS(X) {            \
    CHECKSUM(X "_min_endstop"),           \
    CHECKSUM(X "_max_endstop"),           \
    CHECKSUM(X "_max_travel"),            \
    CHECKSUM(X "_fast_homing_rate_mm_s"), \
    CHECKSUM(X "_slow_homing_rate_mm_s"), \
    CHECKSUM(X "_homing_retract_mm"),     \
    CHECKSUM(X "_homing_direction"),      \
    CHECKSUM(X "_min"),                   \
    CHECKSUM(X "_max"),                   \
    CHECKSUM(X "_limit_enable"),          \
}

// checksum defns
enum DEFNS {MIN_PIN, MAX_PIN, MAX_TRAVEL, FAST_RATE, SLOW_RATE, RETRACT, DIRECTION, MIN, MAX, LIMIT, NDEFNS};

// global config settings
#define corexy_homing_checksum           CHECKSUM("corexy_homing")
#define delta_homing_checksum            CHECKSUM("delta_homing")
#define rdelta_homing_checksum           CHECKSUM("rdelta_homing")
#define scara_homing_checksum            CHECKSUM("scara_homing")

#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")
#define endstop_debounce_ms_checksum     CHECKSUM("endstop_debounce_ms")

#define home_z_first_checksum            CHECKSUM("home_z_first")
#define homing_order_checksum            CHECKSUM("homing_order")
#define move_to_origin_checksum          CHECKSUM("move_to_origin_after_home")

#define alpha_trim_checksum              CHECKSUM("alpha_trim_mm")
#define beta_trim_checksum               CHECKSUM("beta_trim_mm")
#define gamma_trim_checksum              CHECKSUM("gamma_trim_mm")

// new config syntax
// endstop.xmin.enable true
// endstop.xmin.pin 1.29
// endstop.xmin.axis X
// endstop.xmin.type min

#define endstop_checksum                   CHECKSUM("endstop")
#define enable_checksum                    CHECKSUM("enable")
#define pin_checksum                       CHECKSUM("pin")
#define axis_checksum                      CHECKSUM("axis")
#define direction_checksum                 CHECKSUM("homing_direction")
#define position_checksum                  CHECKSUM("homing_position")
#define fast_rate_checksum                 CHECKSUM("fast_rate")
#define slow_rate_checksum                 CHECKSUM("slow_rate")
#define max_travel_checksum                CHECKSUM("max_travel")
#define retract_checksum                   CHECKSUM("retract")
#define limit_checksum                     CHECKSUM("limit_enable")

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())



// Homing States
enum STATES {
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
}

void Endstops::on_module_loaded()
{
    // Do not do anything if not enabled or if no pins are defined
    if (THEKERNEL->config->value( endstops_module_enable_checksum )->by_default(false)->as_bool()) {
        if(!load_old_config()) {
            delete this;
            return;
        }

    }else{
        // check for new config syntax
        if(!load_config()) {
            delete this;
            return;
        }
    }

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);


    THEKERNEL->slow_ticker->attach(1000, this, &Endstops::read_endstops);
}

// Get config using old deprecated syntax Does not support ABC
bool Endstops::load_old_config()
{
    uint16_t const checksums[][NDEFNS] = {
        ENDSTOP_CHECKSUMS("alpha"),   // X
        ENDSTOP_CHECKSUMS("beta"),    // Y
        ENDSTOP_CHECKSUMS("gamma")    // Z
    };

    bool limit_enabled= false;
    for (int i = X_AXIS; i <= Z_AXIS; ++i) { // X_AXIS to Z_AXIS
        // pin definitions for X Y Z min/max pins
        for (int j = MIN_PIN; j <= MAX_PIN; ++j) {
            info_t *info= new info_t;
            info->pin.from_string(THEKERNEL->config->value(checksums[i][j])->by_default("nc" )->as_string())->as_input();
            if(!info->pin.connected()){
                // no pin defined try next
                delete info;
                continue;
            }

            // max pins have MSB set so 0x01 is Y_MIN and 0x81 is Y_MAX
            // enter into endstop map
            uint8_t key= (j == MAX_PIN) ? 0x80 | i : i;
            endstops[key]= info;

            // init struct
            info->home_offset= 0;
            info->homed= false;
            info->debounce= 0;
            info->axis= 'X'+i;
            info->axis_index= i;

            // rates in mm/sec
            info->fast_rate= THEKERNEL->config->value(checksums[i][FAST_RATE])->by_default(100)->as_number();
            info->slow_rate= THEKERNEL->config->value(checksums[i][SLOW_RATE])->by_default(10)->as_number();

            // retract in mm
            info->retract_mm= THEKERNEL->config->value(checksums[i][RETRACT])->by_default(5)->as_number();

            // get homing direction and convert to boolean where true is home to min, and false is home to max
            info->home_direction= THEKERNEL->config->value(checksums[i][DIRECTION])->by_default("home_to_min")->as_string() != "home_to_max";

            // used for homing if it is min pin and direction is min or max pin and direction is max
            info->homing_enabled= ( (info->home_direction && j == MIN_PIN) || (!info->home_direction && j == MAX_PIN) );

            // homing cartesian position
            info->homing_position= info->home_direction ? THEKERNEL->config->value(checksums[i][MIN])->by_default(0)->as_number() : THEKERNEL->config->value(checksums[i][MAX])->by_default(200)->as_number();

            // used to set maximum movement on homing, set by alpha_max_travel if defined
            info->max_travel= THEKERNEL->config->value(checksums[i][MAX_TRAVEL])->by_default(500)->as_number();

            // limits enabled
            info->limit_enable= THEKERNEL->config->value(checksums[i][LIMIT])->by_default(false)->as_bool();
            limit_enabled |= info->limit_enable;
        }
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    get_global_configs();

    if(limit_enabled) {
        register_for_event(ON_IDLE);
    }

    // sanity check for deltas
    /*
    if(this->is_delta || this->is_rdelta) {
        // some things must be the same or they will die, so force it here to avoid config errors
        this->fast_rates[1] = this->fast_rates[2] = this->fast_rates[0];
        this->slow_rates[1] = this->slow_rates[2] = this->slow_rates[0];
        this->retract_mm[1] = this->retract_mm[2] = this->retract_mm[0];
        this->home_direction[1] = this->home_direction[2] = this->home_direction[0];
        // NOTE homing_position for rdelta is the angle of the actuator not the cartesian position
        if(!this->is_rdelta) this->homing_position[0] = this->homing_position[1] = 0;
    }
    */

    return true;
}

// Get config using new syntax supports ABC
bool Endstops::load_config()
{
    bool limit_enabled= false;
    // iterate over all endstop.*.*
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, endstop_checksum);
    for(auto cs : modules ) {
        if(!THEKERNEL->config->value(endstop_checksum, cs, enable_checksum )->as_bool()) continue;
        info_t *info= new info_t;
        info->pin.from_string(THEKERNEL->config->value(endstop_checksum, cs, pin_checksum)->by_default("nc" )->as_string())->as_input();
        if(!info->pin.connected()){
            // no pin defined try next
            delete info;
            continue;
        }

        string axis= THEKERNEL->config->value(endstop_checksum, cs, axis_checksum)->by_default("" )->as_string();
        if(axis.empty()){
            // axis is required
            delete info;
            continue;
        }

        int i;
        switch(toupper(axis[0])) {
            case 'X': i= X_AXIS; break;
            case 'Y': i= Y_AXIS; break;
            case 'Z': i= Z_AXIS; break;
            case 'A': i= A_AXIS; break;
            case 'B': i= B_AXIS; break;
            case 'C': i= C_AXIS; break;
            default: // not a recognized axis
                delete info;
                continue;
        }

        // if set to none it means not used for homing (maybe limit only)
        // home_to_min or home_to_max
        string direction= THEKERNEL->config->value(endstop_checksum, cs, direction_checksum)->by_default("none")->as_string();
        info->homing_enabled = (direction != "none");

        // max pins have MSB set so 0x01 is Y_MIN and 0x81 is Y_MAX
        // enter into endstop map
        uint8_t key= (direction == "home_to_min") ? i : 0x80|i;
        endstops[key]= info;

        // init struct
        info->home_offset= 0;
        info->homed= false;
        info->debounce= 0;
        info->axis= toupper(axis[0]);
        info->axis_index= i;

        // rates in mm/sec
        info->fast_rate= THEKERNEL->config->value(endstop_checksum, cs, fast_rate_checksum)->by_default(100)->as_number();
        info->slow_rate= THEKERNEL->config->value(endstop_checksum, cs, slow_rate_checksum)->by_default(10)->as_number();

        // retract in mm
        info->retract_mm= THEKERNEL->config->value(endstop_checksum, cs, retract_checksum)->by_default(5)->as_number();

        // homing direction and convert to boolean where true is home to min, and false is home to max
        info->home_direction=  direction == "home_to_min";

        // homing cartesian position
        info->homing_position= THEKERNEL->config->value(endstop_checksum, cs, position_checksum)->by_default(info->home_direction ? 0 : 200)->as_number();

        // used to set maximum movement on homing, set by max_travel if defined
        info->max_travel= THEKERNEL->config->value(endstop_checksum, cs, max_travel_checksum)->by_default(500)->as_number();

        // limits enabled
        info->limit_enable= THEKERNEL->config->value(endstop_checksum, cs, limit_checksum)->by_default(false)->as_bool();
        limit_enabled |= info->limit_enable;
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    // sets some endstop global configs applicable to all endstops
    get_global_configs();

    if(limit_enabled) {
        register_for_event(ON_IDLE);
    }
    return true;
}

void Endstops::get_global_configs()
{
    // NOTE the debounce count is in milliseconds so probably does not need to beset anymore
    this->debounce_ms= THEKERNEL->config->value(endstop_debounce_ms_checksum)->by_default(0)->as_number();
    this->debounce_count= THEKERNEL->config->value(endstop_debounce_count_checksum)->by_default(100)->as_number();

    this->is_corexy= THEKERNEL->config->value(corexy_homing_checksum)->by_default(false)->as_bool();
    this->is_delta=  THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_rdelta= THEKERNEL->config->value(rdelta_homing_checksum)->by_default(false)->as_bool();
    this->is_scara=  THEKERNEL->config->value(scara_homing_checksum)->by_default(false)->as_bool();

    this->home_z_first= THEKERNEL->config->value(home_z_first_checksum)->by_default(false)->as_bool();

    this->trim_mm[0] = THEKERNEL->config->value(alpha_trim_checksum )->by_default(0  )->as_number();
    this->trim_mm[1] = THEKERNEL->config->value(beta_trim_checksum  )->by_default(0  )->as_number();
    this->trim_mm[2] = THEKERNEL->config->value(gamma_trim_checksum )->by_default(0  )->as_number();

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

    // set to true by default for deltas due to trim, false on cartesians
    this->move_to_origin_after_home = THEKERNEL->config->value(move_to_origin_checksum)->by_default(is_delta)->as_bool();
}

bool Endstops::debounced_get(uint8_t pin)
{
    auto p= endstops.find(pin);
    if(p == endstops.end()) return false;
    uint8_t debounce = 0;
    while(p->second->pin.get()) {
        if ( ++debounce >= this->debounce_count ) {
            // pin triggered
            return true;
        }
    }
    return false;
}

// only called if limits are enabled
void Endstops::on_idle(void *argument)
{
    if(this->status == LIMIT_TRIGGERED) {
        // if we were in limit triggered see if it has been cleared
        for(auto& i : endstops) {
            if(i.second->limit_enable) {
                if(i.second->pin.get()) {
                    // still triggered, so exit
                    i.second->debounce = 0;
                    return;
                }

                if(i.second->debounce++ > 10) { // can use less as it calls on_idle in between
                    // clear the state
                    this->status = NOT_HOMING;
                }
            }
        }
        return;

    } else if(this->status != NOT_HOMING) {
        // don't check while homing
        return;
    }

    for(auto& i : endstops) {
        if(i.second->limit_enable && STEPPER[i.second->axis_index]->is_moving()) {
            // check min and max endstops
            if(debounced_get(i.first)) {
                // endstop triggered
                string name;
                name.append(1, i.second->axis).append(i.second->home_direction ? "_min" : "_max");
                THEKERNEL->streams->printf("Limit switch %s was hit - reset or M999 required\n", name.c_str());
                this->status = LIMIT_TRIGGERED;
                i.second->debounce= 0;
                // disables heaters and motors, ignores incoming Gcode and flushes block queue
                THEKERNEL->call_event(ON_HALT, nullptr);
                return;
            }
        }
    }
}

// if limit switches are enabled, then we must move off of the endstop otherwise we won't be able to move
// checks if triggered and only backs off if triggered
void Endstops::back_off_home(axis_bitmap_t axis)
{
    std::vector<std::pair<char, float>> params;
    this->status = BACK_OFF_HOME;

    float slow_rate= NAN; // default mm/sec

    // these are handled differently
    if(is_delta) {
        // Move off of the endstop using a regular relative move in Z only
        auto e= endstops.find(Z_AXIS | 0x80); // ZMAX endstop
        if(e != endstops.end()) {
            params.push_back({'Z', e->second->retract_mm * (e->second->home_direction ? 1 : -1)});
            slow_rate= e->second->slow_rate;
        }
    } else {
        // cartesians, concatenate all the moves we need to do into one gcode
        for( auto& e : endstops) {
            if(!axis[e.second->axis_index]) continue; // only for axes we asked to move

            // if not triggered no need to move off
            if(e.second->limit_enable && debounced_get(e.first)) {
                char ax= e.second->axis;
                params.push_back({ax, e.second->retract_mm * (e.second->home_direction ? 1 : -1)});
                // select slowest of them all
                slow_rate= isnan(slow_rate) ? e.second->slow_rate : std::min(slow_rate, e.second->slow_rate);
            }
        }
    }

    if(!params.empty()) {
        // Move off of the endstop using a regular relative move
        params.insert(params.begin(), {'G', 0});
        // use X slow rate to move, Z should have a max speed set anyway
        params.push_back({'F', slow_rate * 60.0F});
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
void Endstops::move_to_origin(axis_bitmap_t axis)
{
    if(!is_delta && (!axis[X_AXIS] || !axis[Y_AXIS])) return; // ignore if X and Y not homing, unless delta

    // Do we need to check if we are already at 0,0? probably not as the G0 will not do anything if we are
    // float pos[3]; THEROBOT->get_axis_position(pos); if(pos[0] == 0 && pos[1] == 0) return;

    this->status = MOVE_TO_ORIGIN;
    // Move to center using a regular move, use slower of X and Y fast rate
    //float rate = std::min(this->fast_rates[0], this->fast_rates[1]) * 60.0F;
    char buf[32];
    THEROBOT->push_state();
    THEROBOT->inch_mode = false;     // needs to be in mm
    THEROBOT->absolute_mode = true;
    //snprintf(buf, sizeof(buf), "G53 G0 X0 Y0 F%1.4f", rate); // must use machine coordinates in case G92 or WCS is in effect
    snprintf(buf, sizeof(buf), "G53 G0 X0 Y0"); // must use machine coordinates in case G92 or WCS is in effect
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
        // check each endstop
        for(auto& e : endstops) { // check all endstops min and max
            if(!e.second->homing_enabled) continue; // ignore if not a homing endstop
            int m= e.second->axis_index;
            if(STEPPER[m]->is_moving()) {
                // if it is moving then we check the associated endstop, and debounce it
                if(e.second->pin.get()) {
                    if(e.second->debounce < debounce_ms) {
                        e.second->debounce++;
                    } else {
                        // we signal the motor to stop, which will preempt any moves on that axis
                        STEPPER[m]->stop_moving();
                    }

                } else {
                    // The endstop was not hit yet
                    e.second->debounce= 0;
                }
            }
        }

    } else {
        // corexy is different as the actuators are not directly related to the XY axis
        // so we check the axis that is currently homing then stop all motors
        for(auto& e : endstops) { // check all endstops min and max
            if(!e.second->homing_enabled) continue; // ignore if not a homing endstop
            int m= e.second->axis_index;
            if(axis_to_home[m]) {
                if(e.second->pin.get()) {
                    if(e.second->debounce < debounce_ms) {
                        e.second->debounce++;
                    } else {
                        // we signal all the motors to stop, as on corexy X and Y motors will move for X and Y axis homing and we only hom eone axis at a time
                        STEPPER[X_AXIS]->stop_moving();
                        STEPPER[Y_AXIS]->stop_moving();
                        STEPPER[Z_AXIS]->stop_moving();
                    }

                } else {
                    // The endstop was not hit yet
                    e.second->debounce= 0;
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
        float delta[3] {alpha_max, beta_max, 0};
        if(this->home_direction[X_AXIS]) delta[X_AXIS]= -delta[X_AXIS];
        if(this->home_direction[Y_AXIS]) delta[Y_AXIS]= -delta[Y_AXIS];
        float feed_rate = std::min(fast_rates[X_AXIS], fast_rates[Y_AXIS]);
        THEROBOT->delta_move(delta, feed_rate, 3);

    } else if(axis_to_home[X_AXIS]) {
        // now home X only
        float delta[3] {alpha_max, 0, 0};
        if(this->home_direction[X_AXIS]) delta[X_AXIS]= -delta[X_AXIS];
        THEROBOT->delta_move(delta, fast_rates[X_AXIS], 3);

    } else if(axis_to_home[Y_AXIS]) {
        // now home Y only
        float delta[3] {0, beta_max, 0};
        if(this->home_direction[Y_AXIS]) delta[Y_AXIS]= -delta[Y_AXIS];
        THEROBOT->delta_move(delta, fast_rates[Y_AXIS], 3);
    }

    // Wait for axis to have homed
    THECONVEYOR->wait_for_idle();
}

void Endstops::home(axis_bitmap_t a)
{
    // reset debounce counts for all endstops
    for(auto& e : endstops) {
       e.second->debounce= 0;
    }

    // turn off any compensation transform so Z does not move as XY home
    auto savect= THEROBOT->compensationTransform;
    THEROBOT->compensationTransform= nullptr;

    this->axis_to_home= a;

    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;

    THEROBOT->disable_segmentation= true; // we must disable segmentation as this won't work with it enabled

    if(!home_z_first) home_xy();

    if(axis_to_home[Z_AXIS]) {
        // now home z
        float delta[3] {0, 0, gamma_max}; // we go the max z
        if(this->home_direction[Z_AXIS]) delta[Z_AXIS]= -delta[Z_AXIS];
        THEROBOT->delta_move(delta, fast_rates[Z_AXIS], 3);
        // wait for Z
        THECONVEYOR->wait_for_idle();
    }

    if(home_z_first) home_xy();

    //TODO need to add BC
    if(axis_to_home[A_AXIS]) {
        // now home A
        float delta[4] {0, 0, 0, epsilon_max}; // we go the max A
        if(this->home_direction[A_AXIS]) delta[A_AXIS]= -delta[A_AXIS];
        THEROBOT->delta_move(delta, fast_rates[A_AXIS], 4);
        // wait for Z
        THECONVEYOR->wait_for_idle();
    }


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
        THEROBOT->push_state();
        THEROBOT->inch_mode = false;     // needs to be in mm
        THEROBOT->absolute_mode = true;
        char buf[32];
        snprintf(buf, sizeof(buf), "G53 G0 X%f Y%f", saved_position[X_AXIS], saved_position[Y_AXIS]); // must use machine coordinates in case G92 or WCS is in effect
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); // as it is a multi G code command
        // Wait for above to finish
        THECONVEYOR->wait_for_idle();
        THEROBOT->pop_state();
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
            homed.set();
        } else {
            // do a manual homing based on given coordinates, no endstops required
            if(gcode->has_letter('X')){ THEROBOT->reset_axis_position(gcode->get_value('X'), X_AXIS); homed.set(X_AXIS); }
            if(gcode->has_letter('Y')){ THEROBOT->reset_axis_position(gcode->get_value('Y'), Y_AXIS); homed.set(Y_AXIS); }
            if(gcode->has_letter('Z')){ THEROBOT->reset_axis_position(gcode->get_value('Z'), Z_AXIS); homed.set(Z_AXIS); }
        }
        return;

    } else if(gcode->subcode == 4) { // G28.4 is a smoothie special it sets manual homing based on the actuator position (used for rotary delta)
        // do a manual homing based on given coordinates, no endstops required
        ActuatorCoordinates ac{NAN, NAN, NAN};
        if(gcode->has_letter('X')){ ac[0] =  gcode->get_value('X'); homed.set(X_AXIS); }
        if(gcode->has_letter('Y')){ ac[1] =  gcode->get_value('Y'); homed.set(Y_AXIS); }
        if(gcode->has_letter('Z')){ ac[2] =  gcode->get_value('Z'); homed.set(Z_AXIS); }
        THEROBOT->reset_actuator_position(ac);
        return;

    } else if(gcode->subcode == 5) { // G28.5 is a smoothie special it clears the homed flag for the specified axis, or all if not specifed
        if(gcode->get_num_args() == 0) {
            homed.reset();
        } else {
            if(gcode->has_letter('X')) homed.reset(X_AXIS);
            if(gcode->has_letter('Y')) homed.reset(Y_AXIS);
            if(gcode->has_letter('Z')) homed.reset(Z_AXIS);
        }
        return;

    } else if(gcode->subcode == 6) { // G28.6 is a smoothie special it shows the homing status of each axis
        for (int i = 0; i < 3; ++i) {
            gcode->stream->printf("%c:%d ", 'X'+i, homed.test(i));
        }
        gcode->add_nl= true;
        return;

    } else if(THEKERNEL->is_grbl_mode()) {
        gcode->stream->printf("error:Unsupported command\n");
        return;
    }

    // G28 is received, we have homing to do

    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // deltas always home Z axis only, which moves all three actuators
    bool home_in_z = this->is_delta || this->is_rdelta;

    // figure out which axis to home
    bitset<6> haxis;
    haxis.reset();

    if(!home_in_z) { // ie not a delta
        bool axis_speced = (gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ||
                            gcode->has_letter('A') || gcode->has_letter('B') || gcode->has_letter('C'));
        // only enable homing if the endstop is defined,
        for ( int c = X_AXIS; c <= C_AXIS; c++ ) {
            auto p= pins.find(this->home_direction[c] ? c : 0x80|c);
            if(p != pins.end() && (!axis_speced || gcode->has_letter(axis_letters[c])) ) {
                haxis.set(c);
                // now reset axis to 0 as we do not know what state we are in
                THEROBOT->reset_axis_position(0, c);
            }
        }

    } else {
        // Only Z axis homes (even though all actuators move this is handled by arm solution)
        haxis.set(Z_AXIS);
        // we also set the kinematics to a known good position, this is necessary for a rotary delta, but doesn't hurt for linear delta
        THEROBOT->reset_axis_position(0, 0, 0);
    }

    // do the actual homing
    if(homing_order != 0) {
        // if an order has been specified do it in the specified order
        // homing order is 0b00ccbbaa where aa is 0,1,2 to specify the first axis, bb is the second and cc is the third
        // eg 0b00100001 would be Y X Z, 0b00100100 would be X Y Z
        for (uint8_t m = homing_order; m != 0; m >>= 2) {
            int a= (m & 0x03); // axis to home
            if(haxis[a]) { // if axis is selected to home
                axis_bitmap_t bs;
                bs.set(a);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }

    } else if(is_corexy) {
        // corexy must home each axis individually
        for (int a = X_AXIS; a <= C_AXIS; ++a) {
            if(haxis[a]) {
                axis_bitmap_t bs;
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
        homed.reset();
        return;
    }

    if(home_in_z) { // deltas only
        // Here's where we would have been if the endstops were perfectly trimmed
        // NOTE on a rotary delta home_offset is actuator position in degrees when homed and
        // home_offset is the theta offset for each actuator, so M206 is used to set theta offset for each actuator in degrees
        // FIXME not sure this will work with compensation transforms on.
        float ideal_position[3] = {
            this->homing_position[X_AXIS] + this->home_offset[X_AXIS],
            this->homing_position[Y_AXIS] + this->home_offset[Y_AXIS],
            this->homing_position[Z_AXIS] + this->home_offset[Z_AXIS]
        };

        bool has_endstop_trim = this->is_delta;
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

        homed.set(); // for deltas we say all axis are homed even though it was only Z

    } else {
        // Zero the ax(i/e)s position, add in the home offset
        // NOTE that if compensation is active the Z will be set based on where XY are, so make sure XY are homed first then Z
        // so XY are at a known consistent position.  (especially true if using a proximity probe)
        for ( int c = X_AXIS; c <= C_AXIS; c++ ) {
            if (haxis[c]) { // if we requested this axis to home
                THEROBOT->reset_axis_position(this->homing_position[c] + this->home_offset[c], c);
                // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
                homed.set(c);
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
    // Similar to M206 but sets Homing offsets based on current MCS position
    // Basically it finds the delta between the current MCS position and the requested position and adds it to the homing offset
    // then will not let it be set again until that axis is homed.
    float pos[3];
    THEROBOT->get_axis_position(pos);

    if (gcode->has_letter('X')) {
        if(!homed[X_AXIS]) {
            gcode->stream->printf("error: Axis X must be homed before setting Homing offset\n");
            return;
        }
        home_offset[0] += (THEROBOT->to_millimeters(gcode->get_value('X')) - pos[X_AXIS]);
        homed.reset(X_AXIS); // force it to be homed
    }
    if (gcode->has_letter('Y')) {
        if(!homed[Y_AXIS]) {
            gcode->stream->printf("error: Axis Y must be homed before setting Homing offset\n");
            return;
        }
        home_offset[1] += (THEROBOT->to_millimeters(gcode->get_value('Y')) - pos[Y_AXIS]);
        homed.reset(Y_AXIS); // force it to be homed
    }
    if (gcode->has_letter('Z')) {
        if(!homed[Z_AXIS]) {
            gcode->stream->printf("error: Axis Z must be homed before setting Homing offset\n");
            return;
        }
        home_offset[2] += (THEROBOT->to_millimeters(gcode->get_value('Z')) - pos[Z_AXIS]);
        homed.reset(Z_AXIS); // force it to be homed
    }

    gcode->stream->printf("Homing Offset: X %5.3f Y %5.3f Z %5.3f will take effect next home\n", home_offset[0], home_offset[1], home_offset[2]);
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
                for(auto& p : pins) {
                    if(p.second->connected()) {
                        int i= (p.first&0x80) ? (p.first&0x7F) + 3 : p.first;
                        gcode->stream->printf("%s:%d ", endstop_names[i], p.second->get());
                    }
                }
                gcode->add_nl = true;
            }
            break;

            case 206: // M206 - set homing offset
                if(is_rdelta) return; // RotaryDeltaCalibration module will handle this

                if (gcode->has_letter('X')) home_offset[0] = gcode->get_value('X');
                if (gcode->has_letter('Y')) home_offset[1] = gcode->get_value('Y');
                if (gcode->has_letter('Z')) home_offset[2] = gcode->get_value('Z');
                gcode->stream->printf("X %5.3f Y %5.3f Z %5.3f will take effect next home\n", home_offset[0], home_offset[1], home_offset[2]);
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
