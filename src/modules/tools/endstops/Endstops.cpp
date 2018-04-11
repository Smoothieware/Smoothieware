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
#include <algorithm>

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
// endstop.xmin.homing_direction home_to_min

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
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset= 0;
        hinfo.homed= false;
        hinfo.axis= 'X'+i;
        hinfo.axis_index= i;
        hinfo.pin_info= nullptr;

        // rates in mm/sec
        hinfo.fast_rate= THEKERNEL->config->value(checksums[i][FAST_RATE])->by_default(100)->as_number();
        hinfo.slow_rate= THEKERNEL->config->value(checksums[i][SLOW_RATE])->by_default(10)->as_number();

        // retract in mm
        hinfo.retract= THEKERNEL->config->value(checksums[i][RETRACT])->by_default(5)->as_number();

        // get homing direction and convert to boolean where true is home to min, and false is home to max
        hinfo.home_direction= THEKERNEL->config->value(checksums[i][DIRECTION])->by_default("home_to_min")->as_string() != "home_to_max";

        // homing cartesian position
        hinfo.homing_position= hinfo.home_direction ? THEKERNEL->config->value(checksums[i][MIN])->by_default(0)->as_number() : THEKERNEL->config->value(checksums[i][MAX])->by_default(200)->as_number();

        // used to set maximum movement on homing, set by alpha_max_travel if defined
        hinfo.max_travel= THEKERNEL->config->value(checksums[i][MAX_TRAVEL])->by_default(500)->as_number();


        // pin definitions for endstop pins
        for (int j = MIN_PIN; j <= MAX_PIN; ++j) {
            endstop_info_t *info= new endstop_info_t;
            info->pin.from_string(THEKERNEL->config->value(checksums[i][j])->by_default("nc" )->as_string())->as_input();
            if(!info->pin.connected()){
                // no pin defined try next
                delete info;
                continue;
            }

            // enter into endstop array
            endstops.push_back(info);

            // add index to the homing struct if this is the one used for homing
            if((hinfo.home_direction && j == MIN_PIN) || (!hinfo.home_direction && j == MAX_PIN)) hinfo.pin_info= info;

            // init struct
            info->debounce= 0;
            info->axis= 'X'+i;
            info->axis_index= i;

            // limits enabled
            info->limit_enable= THEKERNEL->config->value(checksums[i][LIMIT])->by_default(false)->as_bool();
            limit_enabled |= info->limit_enable;
        }

        homing_axis.push_back(hinfo);
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

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
    size_t max_index= 0;

    std::array<homing_info_t, k_max_actuators> temp_axis_array; // needs to be at least XYZ, but allow for ABC
    {
        homing_info_t t;
        t.axis= 0;
        t.axis_index= 0;
        t.pin_info= nullptr;

        temp_axis_array.fill(t);
    }

    // iterate over all endstop.*.*
    std::vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, endstop_checksum);
    for(auto cs : modules ) {
        if(!THEKERNEL->config->value(endstop_checksum, cs, enable_checksum )->as_bool()) continue;

        endstop_info_t *pin_info= new endstop_info_t;
        pin_info->pin.from_string(THEKERNEL->config->value(endstop_checksum, cs, pin_checksum)->by_default("nc" )->as_string())->as_input();
        if(!pin_info->pin.connected()){
            // no pin defined try next
            delete pin_info;
            continue;
        }

        string axis= THEKERNEL->config->value(endstop_checksum, cs, axis_checksum)->by_default("")->as_string();
        if(axis.empty()){
            // axis is required
            delete pin_info;
            continue;
        }

        size_t i;
        switch(toupper(axis[0])) {
            case 'X': i= X_AXIS; break;
            case 'Y': i= Y_AXIS; break;
            case 'Z': i= Z_AXIS; break;
            case 'A': i= A_AXIS; break;
            case 'B': i= B_AXIS; break;
            case 'C': i= C_AXIS; break;
            default: // not a recognized axis
                delete pin_info;
                continue;
        }

        // check we are not going above the number of defined actuators/axis
        if(i >= THEROBOT->get_number_registered_motors()) {
            // too many axis we only have configured n_motors
            THEKERNEL->streams->printf("ERROR: endstop %d is greater than number of defined motors. Endstops disabled\n", i);
            delete pin_info;
            return false;
        }

        // keep track of the maximum index that has been defined
        if(i > max_index) max_index= i;

        // init pin struct
        pin_info->debounce= 0;
        pin_info->axis= toupper(axis[0]);
        pin_info->axis_index= i;

        // are limits enabled
        pin_info->limit_enable= THEKERNEL->config->value(endstop_checksum, cs, limit_checksum)->by_default(false)->as_bool();
        limit_enabled |= pin_info->limit_enable;

        // enter into endstop array
        endstops.push_back(pin_info);

        // if set to none it means not used for homing (maybe limit only) so do not add to the homing array
        string direction= THEKERNEL->config->value(endstop_checksum, cs, direction_checksum)->by_default("none")->as_string();
        if(direction == "none") {
            continue;
        }

        // setup the homing array
        homing_info_t hinfo;

        // init homing struct
        hinfo.home_offset= 0;
        hinfo.homed= false;
        hinfo.axis= toupper(axis[0]);
        hinfo.axis_index= i;
        hinfo.pin_info= pin_info;

        // rates in mm/sec
        hinfo.fast_rate= THEKERNEL->config->value(endstop_checksum, cs, fast_rate_checksum)->by_default(100)->as_number();
        hinfo.slow_rate= THEKERNEL->config->value(endstop_checksum, cs, slow_rate_checksum)->by_default(10)->as_number();

        // retract in mm
        hinfo.retract= THEKERNEL->config->value(endstop_checksum, cs, retract_checksum)->by_default(5)->as_number();

        // homing direction and convert to boolean where true is home to min, and false is home to max
        hinfo.home_direction=  direction == "home_to_min";

        // homing cartesian position
        hinfo.homing_position= THEKERNEL->config->value(endstop_checksum, cs, position_checksum)->by_default(hinfo.home_direction ? 0 : 200)->as_number();

        // used to set maximum movement on homing, set by max_travel if defined
        hinfo.max_travel= THEKERNEL->config->value(endstop_checksum, cs, max_travel_checksum)->by_default(500)->as_number();

        // stick into array in correct place
        temp_axis_array[hinfo.axis_index]= hinfo;
    }

    // if no pins defined then disable the module
    if(endstops.empty()) return false;

    // copy to the homing_axis array, make sure that undefined entries are filled in as well
    // as the order is important and all slots must be filled upto the max_index
    for (size_t i = 0; i < temp_axis_array.size(); ++i) {
        if(temp_axis_array[i].axis == 0) {
            // was not configured above, if it is XYZ then we need to force a dummy entry
            if(i <= Z_AXIS) {
                homing_info_t t;
                t.axis= 'X' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);

            }else if(i <= max_index) {
                // for instance case where we defined C without A or B
                homing_info_t t;
                t.axis= 'A' + i;
                t.axis_index= i;
                t.pin_info= nullptr; // this tells it that it cannot be used for homing
                homing_axis.push_back(t);
            }

        }else{
            homing_axis.push_back(temp_axis_array[i]);
        }
    }

    // saves some memory
    homing_axis.shrink_to_fit();
    endstops.shrink_to_fit();

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

    this->trim_mm[0] = THEKERNEL->config->value(alpha_trim_checksum)->by_default(0)->as_number();
    this->trim_mm[1] = THEKERNEL->config->value(beta_trim_checksum)->by_default(0)->as_number();
    this->trim_mm[2] = THEKERNEL->config->value(gamma_trim_checksum)->by_default(0)->as_number();

    // see if an order has been specified, must be three or more characters, XYZABC or ABYXZ etc
    string order = THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order = 0;
    if(order.size() >= 3 && order.size() <= homing_axis.size() && !(this->is_delta || this->is_rdelta)) {
        int shift = 0;
        for(auto c : order) {
            char n= toupper(c);
            uint32_t i = n >= 'X' ? n - 'X' : n - 'A' + 3;
            i += 1; // So X is 1
            if(i > 6) { // bad value
                this->homing_order = 0;
                break;
            }
            homing_order |= (i << shift);
            shift += 3;
        }
    }

    // set to true by default for deltas due to trim, false on cartesians
    this->move_to_origin_after_home = THEKERNEL->config->value(move_to_origin_checksum)->by_default(is_delta)->as_bool();
}

bool Endstops::debounced_get(Pin *pin)
{
    if(pin == nullptr) return false;
    uint8_t debounce = 0;
    while(pin->get()) {
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
            if(i->limit_enable) {
                if(i->pin.get()) {
                    // still triggered, so exit
                    i->debounce = 0;
                    return;
                }

                if(i->debounce++ > debounce_count) { // can use less as it calls on_idle in between
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
        if(i->limit_enable && STEPPER[i->axis_index]->is_moving()) {
            // check min and max endstops
            if(debounced_get(&i->pin)) {
                // endstop triggered
                if(!THEKERNEL->is_grbl_mode()) {
                    THEKERNEL->streams->printf("Limit switch %c%c was hit - reset or M999 required\n", STEPPER[i->axis_index]->which_direction() ? '-' : '+', i->axis);
                }else{
                    THEKERNEL->streams->printf("ALARM: Hard limit %c%c\n", STEPPER[i->axis_index]->which_direction() ? '-' : '+', i->axis);
                }
                this->status = LIMIT_TRIGGERED;
                i->debounce= 0;
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
        params.push_back({'Z', THEROBOT->from_millimeters(homing_axis[Z_AXIS].retract * (homing_axis[Z_AXIS].home_direction ? 1 : -1))});
        slow_rate= homing_axis[Z_AXIS].slow_rate;

    } else {
        // cartesians concatenate all the moves we need to do into one gcode
        for( auto& e : homing_axis) {
            if(!axis[e.axis_index]) continue; // only for axes we asked to move

            // if not triggered no need to move off
            if(e.pin_info != nullptr && e.pin_info->limit_enable && debounced_get(&e.pin_info->pin)) {
                char ax= e.axis;
                params.push_back({ax, THEROBOT->from_millimeters(e.retract * (e.home_direction ? 1 : -1))});
                // select slowest of them all
                slow_rate= isnan(slow_rate) ? e.slow_rate : std::min(slow_rate, e.slow_rate);
            }
        }
    }

    if(!params.empty()) {
        // Move off of the endstop using a regular relative move
        params.insert(params.begin(), {'G', 0});
        // use X slow rate to move, Z should have a max speed set anyway
        params.push_back({'F', THEROBOT->from_millimeters(slow_rate * 60.0F)});
        char gcode_buf[64];
        append_parameters(gcode_buf, params, sizeof(gcode_buf));
        Gcode gc(gcode_buf, &(StreamOutput::NullStream));
        THEROBOT->push_state();
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
    // Move to center using a regular move, use slower of X and Y fast rate in mm/sec
    float rate = std::min(homing_axis[X_AXIS].fast_rate, homing_axis[Y_AXIS].fast_rate) * 60.0F;
    char buf[32];
    THEROBOT->push_state();
    THEROBOT->absolute_mode = true;
    snprintf(buf, sizeof(buf), "G53 G0 X0 Y0 F%1.4f", THEROBOT->from_millimeters(rate)); // must use machine coordinates in case G92 or WCS is in effect
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

    // check each homing endstop
    for(auto& e : homing_axis) { // check all axis homing endstops
        if(e.pin_info == nullptr) continue; // ignore if not a homing endstop
        int m= e.axis_index;

        // for corexy homing in X or Y we must only check the associated endstop, works as we only home one axis at a time for corexy
        if(is_corexy && (m == X_AXIS || m == Y_AXIS) && !axis_to_home[m]) continue;

        if(STEPPER[m]->is_moving()) {
            // if it is moving then we check the associated endstop, and debounce it
            if(e.pin_info->pin.get()) {
                if(e.pin_info->debounce < debounce_ms) {
                    e.pin_info->debounce++;

                } else {
                    if(is_corexy && (m == X_AXIS || m == Y_AXIS)) {
                        // corexy when moving in X or Y we need to stop both the X and Y motors
                        STEPPER[X_AXIS]->stop_moving();
                        STEPPER[Y_AXIS]->stop_moving();

                    }else{
                        // we signal the motor to stop, which will preempt any moves on that axis
                        STEPPER[m]->stop_moving();
                    }
                    e.pin_info->triggered= true;
                }

            } else {
                // The endstop was not hit yet
                e.pin_info->debounce= 0;
            }
        }
    }

    return 0;
}

void Endstops::home_xy()
{
    if(axis_to_home[X_AXIS] && axis_to_home[Y_AXIS]) {
        // Home XY first so as not to slow them down by homing Z at the same time
        float delta[3] {homing_axis[X_AXIS].max_travel, homing_axis[Y_AXIS].max_travel, 0};
        if(homing_axis[X_AXIS].home_direction) delta[X_AXIS]= -delta[X_AXIS];
        if(homing_axis[Y_AXIS].home_direction) delta[Y_AXIS]= -delta[Y_AXIS];
        float feed_rate = std::min(homing_axis[X_AXIS].fast_rate, homing_axis[Y_AXIS].fast_rate);
        THEROBOT->delta_move(delta, feed_rate, 3);

    } else if(axis_to_home[X_AXIS]) {
        // now home X only
        float delta[3] {homing_axis[X_AXIS].max_travel, 0, 0};
        if(homing_axis[X_AXIS].home_direction) delta[X_AXIS]= -delta[X_AXIS];
        THEROBOT->delta_move(delta, homing_axis[X_AXIS].fast_rate, 3);

    } else if(axis_to_home[Y_AXIS]) {
        // now home Y only
        float delta[3] {0,  homing_axis[Y_AXIS].max_travel, 0};
        if(homing_axis[Y_AXIS].home_direction) delta[Y_AXIS]= -delta[Y_AXIS];
        THEROBOT->delta_move(delta, homing_axis[Y_AXIS].fast_rate, 3);
    }

    // Wait for axis to have homed
    THECONVEYOR->wait_for_idle();
}

void Endstops::home(axis_bitmap_t a)
{
    // reset debounce counts for all endstops
    for(auto& e : endstops) {
       e->debounce= 0;
       e->triggered= false;
    }

    if (is_scara) {
        THEROBOT->disable_arm_solution = true;  // Polar bots has to home in the actuator space.  Arm solution disabled.
    }

    this->axis_to_home= a;

    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;

    THEROBOT->disable_segmentation= true; // we must disable segmentation as this won't work with it enabled

    if(!home_z_first) home_xy();

    if(axis_to_home[Z_AXIS]) {
        // now home z
        float delta[3] {0, 0, homing_axis[Z_AXIS].max_travel}; // we go the max z
        if(homing_axis[Z_AXIS].home_direction) delta[Z_AXIS]= -delta[Z_AXIS];
        THEROBOT->delta_move(delta, homing_axis[Z_AXIS].fast_rate, 3);
        // wait for Z
        THECONVEYOR->wait_for_idle();
    }

    if(home_z_first) home_xy();

    // potentially home A B and C individually
    if(homing_axis.size() > 3){
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i]) {
                // now home A B or C
                float delta[i+1];
                for (size_t j = 0; j <= i; ++j) delta[j]= 0;
                delta[i]= homing_axis[i].max_travel; // we go the max
                if(homing_axis[i].home_direction) delta[i]= -delta[i];
                THEROBOT->delta_move(delta, homing_axis[i].fast_rate, i+1);
                // wait for it
                THECONVEYOR->wait_for_idle();
            }
        }
    }

    // check that the endstops were hit and it did not stop short for some reason
    // if the endstop is not triggered then enter ALARM state
    // with deltas we check all three axis were triggered, but at least one of XYZ must be set to home
    if(axis_to_home[X_AXIS] || axis_to_home[Y_AXIS] || axis_to_home[Z_AXIS]) {
        for (size_t i = X_AXIS; i <= Z_AXIS; ++i) {
            if((axis_to_home[i] || this->is_delta || this->is_rdelta) && !homing_axis[i].pin_info->triggered) {
                this->status = NOT_HOMING;
                THEKERNEL->call_event(ON_HALT, nullptr);
                return;
            }
        }
    }

    // also check ABC
    if(homing_axis.size() > 3){
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            if(axis_to_home[i] && !homing_axis[i].pin_info->triggered) {
                this->status = NOT_HOMING;
                THEKERNEL->call_event(ON_HALT, nullptr);
                return;
            }
        }
    }

    if (!is_scara) {
        // Only for non polar bots
        // we did not complete movement the full distance if we hit the endstops
        // TODO Maybe only reset axis involved in the homing cycle
        THEROBOT->reset_position_from_current_actuator_position();
    }

    // Move back a small distance for all homing axis
    this->status = MOVING_BACK;
    float delta[homing_axis.size()];
    for (size_t i = 0; i < homing_axis.size(); ++i) delta[i]= 0;

    // use minimum feed rate of all axes that are being homed (sub optimal, but necessary)
    float feed_rate= homing_axis[X_AXIS].slow_rate;
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract;
            if(!i.home_direction) delta[c]= -delta[c];
            feed_rate= std::min(i.slow_rate, feed_rate);
        }
    }

    THEROBOT->delta_move(delta, feed_rate, homing_axis.size());
    // wait until finished
    THECONVEYOR->wait_for_idle();

    // Start moving the axes towards the endstops slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    for (auto& i : homing_axis) {
        int c= i.axis_index;
        if(axis_to_home[c]) {
            delta[c]= i.retract*2; // move further than we moved off to make sure we hit it cleanly
            if(i.home_direction) delta[c]= -delta[c];
        }else{
            delta[c]= 0;
        }
    }
    THEROBOT->delta_move(delta, feed_rate, homing_axis.size());
    // wait until finished
    THECONVEYOR->wait_for_idle();

    // we did not complete movement the full distance if we hit the endstops
    // TODO Maybe only reset axis involved in the homing cycle
    THEROBOT->reset_position_from_current_actuator_position();

    THEROBOT->disable_segmentation= false;
    if (is_scara) {
        THEROBOT->disable_arm_solution = false;  // Arm solution enabled again.
    }

    this->status = NOT_HOMING;
}

void Endstops::process_home_command(Gcode* gcode)
{
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // turn off any compensation transform so Z does not move as XY home
    auto savect= THEROBOT->compensationTransform;
    THEROBOT->compensationTransform= nullptr;

    // deltas always home Z axis only, which moves all three actuators
    bool home_in_z_only = this->is_delta || this->is_rdelta;

    // figure out which axis to home
    axis_bitmap_t haxis;
    haxis.reset();

    bool axis_speced = (gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ||
                        gcode->has_letter('A') || gcode->has_letter('B') || gcode->has_letter('C'));

    if(!home_in_z_only) { // ie not a delta
        for (auto &p : homing_axis) {
            // only enable homing if the endstop is defined,
            if(p.pin_info == nullptr) continue;
            if(!axis_speced || gcode->has_letter(p.axis)) {
                haxis.set(p.axis_index);
                // now reset axis to 0 as we do not know what state we are in
                if (!is_scara) {
                    THEROBOT->reset_axis_position(0, p.axis_index);
                } else {
                    // SCARA resets arms to plausable minimum angles
                    THEROBOT->reset_axis_position(-30,30,0); // angles set into axis space for homing.
                }
            }
        }

    } else {
        bool home_z= !axis_speced || gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z');

        // if we specified an axis we check ABC
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            auto &p= homing_axis[i];
            if(p.pin_info == nullptr) continue;
            if(!axis_speced || gcode->has_letter(p.axis)) haxis.set(p.axis_index);
        }

        if(home_z){
            // Only Z axis homes (even though all actuators move this is handled by arm solution)
            haxis.set(Z_AXIS);
            // we also set the kinematics to a known good position, this is necessary for a rotary delta, but doesn't hurt for linear delta
            THEROBOT->reset_axis_position(0, 0, 0);
        }
    }

    if(haxis.none()) {
        THEKERNEL->streams->printf("WARNING: Nothing to home\n");
        return;
    }

    // do the actual homing
    if(homing_order != 0 && !is_scara) {
        // if an order has been specified do it in the specified order
        // homing order is 0bfffeeedddcccbbbaaa where aaa is 1,2,3,4,5,6 to specify the first axis (XYZABC), bbb is the second and ccc is the third etc
        // eg 0b0101011001010 would be Y X Z A, 011 010 001 100 101 would be  B A X Y Z
        for (uint32_t m = homing_order; m != 0; m >>= 3) {
            uint32_t a= (m & 0x07)-1; // axis to home
            if(a < homing_axis.size() && haxis[a]) { // if axis is selected to home
                axis_bitmap_t bs;
                bs.set(a);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }

    } else if(is_corexy) {
        // corexy must home each axis individually
        for (auto &p : homing_axis) {
            if(haxis[p.axis_index]) {
                axis_bitmap_t bs;
                bs.set(p.axis_index);
                home(bs);
            }
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }

    } else {
        // they could all home at the same time
        home(haxis);
    }

    // restore compensationTransform
    THEROBOT->compensationTransform= savect;

    // check if on_halt (eg kill or fail)
    if(THEKERNEL->is_halted()) {
        if(!THEKERNEL->is_grbl_mode()) {
            THEKERNEL->streams->printf("ERROR: Homing cycle failed - check the max_travel settings\n");
        }else{
            THEKERNEL->streams->printf("ALARM: Homing fail\n");
        }
        // clear all the homed flags
        for (auto &p : homing_axis) p.homed= false;
        return;
    }

    if(home_in_z_only || is_scara) { // deltas and scaras only
        // Here's where we would have been if the endstops were perfectly trimmed
        // NOTE on a rotary delta home_offset is actuator position in degrees when homed and
        // home_offset is the theta offset for each actuator, so M206 is used to set theta offset for each actuator in degrees
        // FIXME not sure this will work with compensation transforms on.
        float ideal_position[3] = {
            homing_axis[X_AXIS].homing_position + homing_axis[X_AXIS].home_offset,
            homing_axis[Y_AXIS].homing_position + homing_axis[Y_AXIS].home_offset,
            homing_axis[Z_AXIS].homing_position + homing_axis[Z_AXIS].home_offset
        };

        bool has_endstop_trim = this->is_delta || is_scara;
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
            // Reset the actuator positions to correspond to our real position
            THEROBOT->reset_axis_position(real_position[0], real_position[1], real_position[2]);

        } else {
            // without endstop trim, real_position == ideal_position
            if(is_rdelta) {
                // with a rotary delta we set the actuators angle then use the FK to calculate the resulting cartesian coordinates
                ActuatorCoordinates real_actuator_position = {ideal_position[0], ideal_position[1], ideal_position[2]};
                THEROBOT->reset_actuator_position(real_actuator_position);

            } else {
                // Reset the actuator positions to correspond to our real position
                THEROBOT->reset_axis_position(ideal_position[0], ideal_position[1], ideal_position[2]);
            }
        }

        // for deltas we say all 3 axis are homed even though it was only Z
        homing_axis[X_AXIS].homed= true;
        homing_axis[Y_AXIS].homed= true;
        homing_axis[Z_AXIS].homed= true;

        // if we also homed ABC then we need to reset them
        for (size_t i = A_AXIS; i < homing_axis.size(); ++i) {
            auto &p= homing_axis[i];
            if (haxis[p.axis_index]) { // if we requested this axis to home
                THEROBOT->reset_axis_position(p.homing_position + p.home_offset, p.axis_index);
                // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
                p.homed= true;
            }
        }

    } else {
        // Zero the ax(i/e)s position, add in the home offset
        // NOTE that if compensation is active the Z will be set based on where XY are, so make sure XY are homed first then Z
        // so XY are at a known consistent position.  (especially true if using a proximity probe)
        for (auto &p : homing_axis) {
            if (haxis[p.axis_index]) { // if we requested this axis to home
                THEROBOT->reset_axis_position(p.homing_position + p.home_offset, p.axis_index);
                // set flag indicating axis was homed, it stays set once set until H/W reset or unhomed
                p.homed= true;
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

    } else if(haxis[Z_AXIS] && (this->move_to_origin_after_home || homing_axis[X_AXIS].pin_info->limit_enable)) {
        // deltas are not left at 0,0 because of the trim settings, so move to 0,0 if requested, but we need to back off endstops first
        // also need to back off endstops if limits are enabled
        back_off_home(haxis);
        if(this->move_to_origin_after_home) move_to_origin(haxis);
    }
}

void Endstops::set_homing_offset(Gcode *gcode)
{
    // M306 Similar to M206 but sets Homing offsets based on current MCS position
    // Basically it finds the delta between the current MCS position and the requested position and adds it to the homing offset
    // then will not let it be set again until that axis is homed.
    float pos[3];
    THEROBOT->get_axis_position(pos);

    if (gcode->has_letter('X')) {
        if(!homing_axis[X_AXIS].homed) {
            gcode->stream->printf("error: Axis X must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[X_AXIS].home_offset += (THEROBOT->to_millimeters(gcode->get_value('X')) - pos[X_AXIS]);
        homing_axis[X_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Y')) {
        if(!homing_axis[Y_AXIS].homed) {
            gcode->stream->printf("error: Axis Y must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Y_AXIS].home_offset += (THEROBOT->to_millimeters(gcode->get_value('Y')) - pos[Y_AXIS]);
        homing_axis[Y_AXIS].homed= false; // force it to be homed
    }
    if (gcode->has_letter('Z')) {
        if(!homing_axis[Z_AXIS].homed) {
            gcode->stream->printf("error: Axis Z must be homed before setting Homing offset\n");
            return;
        }
        homing_axis[Z_AXIS].home_offset += (THEROBOT->to_millimeters(gcode->get_value('Z')) - pos[Z_AXIS]);
        homing_axis[Z_AXIS].homed= false; // force it to be homed
    }

    gcode->stream->printf("Homing Offset: X %5.3f Y %5.3f Z %5.3f will take effect next home\n", homing_axis[X_AXIS].home_offset, homing_axis[Y_AXIS].home_offset, homing_axis[Z_AXIS].home_offset);
}

void Endstops::handle_park(Gcode * gcode)
{
    // TODO: spec says if XYZ specified move to them first then move to MCS of specifed axis
    THEROBOT->push_state();
    THEROBOT->absolute_mode = true;
    char buf[32];
    snprintf(buf, sizeof(buf), "G53 G0 X%f Y%f", THEROBOT->from_millimeters(saved_position[X_AXIS]), THEROBOT->from_millimeters(saved_position[Y_AXIS])); // must use machine coordinates in case G92 or WCS is in effect
    struct SerialMessage message;
    message.message = buf;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message ); // as it is a multi G code command
    // Wait for above to finish
    THECONVEYOR->wait_for_idle();
    THEROBOT->pop_state();
}

// parse gcodes
void Endstops::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if ( gcode->has_g && gcode->g == 28) {
        switch(gcode->subcode) {
            case 0: // G28 in grbl mode will do a rapid to the predefined position otherwise it is home command
                if(THEKERNEL->is_grbl_mode()){
                    handle_park(gcode);
                }else{
                    process_home_command(gcode);
                }
                break;

            case 1: // G28.1 set pre defined park position
                // saves current position in absolute machine coordinates
                THEROBOT->get_axis_position(saved_position); // Only XY are used
                // Note the following is only meant to be used for recovering a saved position from config-override
                // Not a standard Gcode and not to be relied on
                if (gcode->has_letter('X')) saved_position[X_AXIS] = gcode->get_value('X');
                if (gcode->has_letter('Y')) saved_position[Y_AXIS] = gcode->get_value('Y');
                break;

            case 2: // G28.2 in grbl mode does homing (triggered by $H), otherwise it moves to the park position
                if(THEKERNEL->is_grbl_mode()) {
                    process_home_command(gcode);
                }else{
                    handle_park(gcode);
                }
                break;

            case 3: // G28.3 is a smoothie special it sets manual homing
                if(gcode->get_num_args() == 0) {
                    for (auto &p : homing_axis) {
                        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                        p.homed= true;
                        THEROBOT->reset_axis_position(0, p.axis_index);
                    }
                } else {
                    // do a manual homing based on given coordinates, no endstops required
                    if(gcode->has_letter('X')){ THEROBOT->reset_axis_position(gcode->get_value('X'), X_AXIS); homing_axis[X_AXIS].homed= true; }
                    if(gcode->has_letter('Y')){ THEROBOT->reset_axis_position(gcode->get_value('Y'), Y_AXIS); homing_axis[Y_AXIS].homed= true; }
                    if(gcode->has_letter('Z')){ THEROBOT->reset_axis_position(gcode->get_value('Z'), Z_AXIS); homing_axis[Z_AXIS].homed= true; }
                    if(homing_axis.size() > A_AXIS && homing_axis[A_AXIS].pin_info != nullptr && gcode->has_letter('A')){ THEROBOT->reset_axis_position(gcode->get_value('A'), A_AXIS); homing_axis[A_AXIS].homed= true; }
                    if(homing_axis.size() > B_AXIS && homing_axis[B_AXIS].pin_info != nullptr && gcode->has_letter('B')){ THEROBOT->reset_axis_position(gcode->get_value('B'), B_AXIS); homing_axis[B_AXIS].homed= true; }
                    if(homing_axis.size() > C_AXIS && homing_axis[C_AXIS].pin_info != nullptr && gcode->has_letter('C')){ THEROBOT->reset_axis_position(gcode->get_value('C'), C_AXIS); homing_axis[C_AXIS].homed= true; }
                }
                break;

            case 4: { // G28.4 is a smoothie special it sets manual homing based on the actuator position (used for rotary delta)
                    // do a manual homing based on given coordinates, no endstops required
                    ActuatorCoordinates ac{NAN, NAN, NAN};
                    if(gcode->has_letter('X')){ ac[0] =  gcode->get_value('X'); homing_axis[X_AXIS].homed= true; }
                    if(gcode->has_letter('Y')){ ac[1] =  gcode->get_value('Y'); homing_axis[Y_AXIS].homed= true; }
                    if(gcode->has_letter('Z')){ ac[2] =  gcode->get_value('Z'); homing_axis[Z_AXIS].homed= true; }
                    THEROBOT->reset_actuator_position(ac);
                }
                break;

            case 5: // G28.5 is a smoothie special it clears the homed flag for the specified axis, or all if not specifed
                if(gcode->get_num_args() == 0) {
                    for (auto &p : homing_axis) p.homed= false;
                } else {
                    if(gcode->has_letter('X')) homing_axis[X_AXIS].homed= false;
                    if(gcode->has_letter('Y')) homing_axis[Y_AXIS].homed= false;
                    if(gcode->has_letter('Z')) homing_axis[Z_AXIS].homed= false;
                    if(homing_axis.size() > A_AXIS && gcode->has_letter('A')) homing_axis[A_AXIS].homed= false;
                    if(homing_axis.size() > B_AXIS && gcode->has_letter('B')) homing_axis[B_AXIS].homed= false;
                    if(homing_axis.size() > C_AXIS && gcode->has_letter('C')) homing_axis[C_AXIS].homed= false;
                }
                break;

            case 6: // G28.6 is a smoothie special it shows the homing status of each axis
                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    gcode->stream->printf("%c:%d ", p.axis, p.homed);
                }
                gcode->add_nl= true;
                break;

            default:
                if(THEKERNEL->is_grbl_mode()) {
                    gcode->stream->printf("error:Unsupported command\n");
                }
                break;
        }

    } else if (gcode->has_m) {

        switch (gcode->m) {
            case 119: {
                for(auto& h : homing_axis) {
                    if(h.pin_info == nullptr) continue; // ignore if not a homing endstop
                    string name;
                    name.append(1, h.axis).append(h.home_direction ? "_min" : "_max");
                    gcode->stream->printf("%s:%d ", name.c_str(), h.pin_info->pin.get());
                }
                gcode->stream->printf("pins- ");
                for(auto& p : endstops) {
                    string str(1, p->axis);
                    if(p->limit_enable) str.append("L");
                    gcode->stream->printf("(%s)P%d.%d:%d ", str.c_str(), p->pin.port_number, p->pin.pin, p->pin.get());
                }
                gcode->add_nl = true;
            }
            break;

            case 206: // M206 - set homing offset
                if(is_rdelta) return; // RotaryDeltaCalibration module will handle this
                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    if (gcode->has_letter(p.axis)) p.home_offset= gcode->get_value(p.axis);
                }

                for (auto &p : homing_axis) {
                    if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                    gcode->stream->printf("%c: %5.3f ", p.axis, p.home_offset);
                }

                gcode->stream->printf(" will take effect next home\n");
                break;

            case 306: // set homing offset based on current position
                if(is_rdelta) return; // RotaryDeltaCalibration module will handle this

                set_homing_offset(gcode);
                break;

            case 500: // save settings
            case 503: // print settings
                if(!is_rdelta) {
                    gcode->stream->printf(";Home offset (mm):\nM206 ");
                    for (auto &p : homing_axis) {
                        if(p.pin_info == nullptr) continue; // ignore if not a homing endstop
                        gcode->stream->printf("%c%1.2f ", p.axis, p.home_offset);
                    }
                    gcode->stream->printf("\n");

                }else{
                    gcode->stream->printf(";Theta offset (degrees):\nM206 A%1.5f B%1.5f C%1.5f\n",
                        homing_axis[X_AXIS].home_offset, homing_axis[Y_AXIS].home_offset, homing_axis[Z_AXIS].home_offset);
                }

                if (this->is_delta || this->is_scara) {
                    gcode->stream->printf(";Trim (mm):\nM666 X%1.3f Y%1.3f Z%1.3f\n", trim_mm[0], trim_mm[1], trim_mm[2]);
                    gcode->stream->printf(";Max Z\nM665 Z%1.3f\n", homing_axis[Z_AXIS].homing_position);
                }
                if(saved_position[X_AXIS] != 0 || saved_position[Y_AXIS] != 0) {
                    gcode->stream->printf(";predefined position:\nG28.1 X%1.4f Y%1.4f\n", saved_position[X_AXIS], saved_position[Y_AXIS]);
                }
                break;

            case 665:
                if (this->is_delta || this->is_scara) { // M665 - set max gamma/z height
                    float gamma_max = homing_axis[Z_AXIS].homing_position;
                    if (gcode->has_letter('Z')) {
                        homing_axis[Z_AXIS].homing_position= gamma_max = gcode->get_value('Z');
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
        // provided by caller
        float *data = static_cast<float *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; ++i) {
            data[i]= homing_axis[i].home_offset;
        }
        pdr->set_taken();

    } else if(pdr->second_element_is(saved_position_checksum)) {
        pdr->set_data_ptr(&this->saved_position);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_homing_status_checksum)) {
        bool *homing = static_cast<bool *>(pdr->get_data_ptr());
        *homing = this->status != NOT_HOMING;
        pdr->set_taken();

    } else if(pdr->second_element_is(get_homed_status_checksum)) {
        bool *homed = static_cast<bool *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; ++i) {
            homed[i]= homing_axis[i].homed;
        }
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
        if(!isnan(t[0])) homing_axis[0].home_offset= t[0];
        if(!isnan(t[1])) homing_axis[1].home_offset= t[1];
        if(!isnan(t[2])) homing_axis[2].home_offset= t[2];
    }
}
