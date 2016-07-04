/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Extruder.h"

#include "libs/Module.h"
#include "libs/Kernel.h"

#include "modules/robot/Conveyor.h"
#include "modules/robot/Block.h"
#include "StepperMotor.h"
#include "SlowTicker.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "StreamOutputPool.h"
#include "ExtruderPublicAccess.h"

#include <mri.h>

// OLD config names for backwards compatibility, NOTE new configs will not be added here
#define extruder_module_enable_checksum      CHECKSUM("extruder_module_enable")
#define extruder_steps_per_mm_checksum       CHECKSUM("extruder_steps_per_mm")
#define extruder_filament_diameter_checksum  CHECKSUM("extruder_filament_diameter")
#define extruder_acceleration_checksum       CHECKSUM("extruder_acceleration")
#define extruder_step_pin_checksum           CHECKSUM("extruder_step_pin")
#define extruder_dir_pin_checksum            CHECKSUM("extruder_dir_pin")
#define extruder_en_pin_checksum             CHECKSUM("extruder_en_pin")
#define extruder_max_speed_checksum          CHECKSUM("extruder_max_speed")
#define extruder_default_feed_rate_checksum  CHECKSUM("extruder_default_feed_rate")

// NEW config names

#define default_feed_rate_checksum           CHECKSUM("default_feed_rate")
#define steps_per_mm_checksum                CHECKSUM("steps_per_mm")
#define filament_diameter_checksum           CHECKSUM("filament_diameter")
#define acceleration_checksum                CHECKSUM("acceleration")
#define step_pin_checksum                    CHECKSUM("step_pin")
#define dir_pin_checksum                     CHECKSUM("dir_pin")
#define en_pin_checksum                      CHECKSUM("en_pin")
#define max_speed_checksum                   CHECKSUM("max_speed")
#define x_offset_checksum                    CHECKSUM("x_offset")
#define y_offset_checksum                    CHECKSUM("y_offset")
#define z_offset_checksum                    CHECKSUM("z_offset")

#define retract_length_checksum              CHECKSUM("retract_length")
#define retract_feedrate_checksum            CHECKSUM("retract_feedrate")
#define retract_recover_length_checksum      CHECKSUM("retract_recover_length")
#define retract_recover_feedrate_checksum    CHECKSUM("retract_recover_feedrate")
#define retract_zlift_length_checksum        CHECKSUM("retract_zlift_length")
#define retract_zlift_feedrate_checksum      CHECKSUM("retract_zlift_feedrate")

#define PI 3.14159265358979F


/*
    As the actual motion is handled by the planner and the stepticker, this module just handles Extruder specific gcodes
    and settings.
    In a multi extruder setting it must be selected to be addressed. (using T0 T1 etc)
*/

Extruder::Extruder( uint16_t config_identifier)
{
    this->selected = false;
    this->identifier = config_identifier;
    this->retracted = false;
    this->volumetric_multiplier = 1.0F;
    this->extruder_multiplier = 1.0F;
    this->stepper_motor = nullptr;
    this->max_volumetric_rate = 0;
    this->g92e0_detected = false;
    memset(this->offset, 0, sizeof(this->offset));
}

Extruder::~Extruder()
{
    delete stepper_motor;
}

void Extruder::on_module_loaded()
{
    // Settings
    this->config_load();

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

// Get config
void Extruder::config_load()
{

    Pin step_pin, dir_pin, en_pin;
    step_pin.from_string( THEKERNEL->config->value(extruder_checksum, this->identifier, step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
    dir_pin.from_string(  THEKERNEL->config->value(extruder_checksum, this->identifier, dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
    en_pin.from_string(   THEKERNEL->config->value(extruder_checksum, this->identifier, en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

    float steps_per_millimeter = THEKERNEL->config->value(extruder_checksum, this->identifier, steps_per_mm_checksum)->by_default(1)->as_number();
    float acceleration         = THEKERNEL->config->value(extruder_checksum, this->identifier, acceleration_checksum)->by_default(1000)->as_number();

    this->offset[X_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, x_offset_checksum          )->by_default(0)->as_number();
    this->offset[Y_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, y_offset_checksum          )->by_default(0)->as_number();
    this->offset[Z_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, z_offset_checksum          )->by_default(0)->as_number();

    this->filament_diameter        = THEKERNEL->config->value(extruder_checksum, this->identifier, filament_diameter_checksum )->by_default(0)->as_number();
    this->retract_length           = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_length_checksum)->by_default(3)->as_number();
    this->retract_feedrate         = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_feedrate_checksum)->by_default(45)->as_number();
    this->retract_recover_length   = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_recover_length_checksum)->by_default(0)->as_number();
    this->retract_recover_feedrate = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_recover_feedrate_checksum)->by_default(8)->as_number();
    this->retract_zlift_length     = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_zlift_length_checksum)->by_default(0)->as_number();
    this->retract_zlift_feedrate   = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_zlift_feedrate_checksum)->by_default(100 * 60)->as_number() / 60.0F; // mm/min

    if(filament_diameter > 0.01F) {
        this->volumetric_multiplier = 1.0F / (powf(this->filament_diameter / 2, 2) * PI);
    }

    // Stepper motor object for the extruder
    stepper_motor = new StepperMotor(step_pin, dir_pin, en_pin);
    motor_id = THEROBOT->register_motor(stepper_motor);

    stepper_motor->set_max_rate(THEKERNEL->config->value(extruder_checksum, this->identifier, max_speed_checksum)->by_default(1000)->as_number());
    stepper_motor->set_acceleration(acceleration);
    stepper_motor->change_steps_per_mm(steps_per_millimeter);
    stepper_motor->set_selected(false); // not selected by default
}

void Extruder::select()
{
    selected = true;
    stepper_motor->set_selected(true);
    // set the function pointer to return the current scaling
    THEROBOT->get_e_scale_fnc = std::bind(&Extruder::get_e_scale, this);
}

void Extruder::deselect()
{
    selected = false;
    stepper_motor->set_selected(false);
    THEROBOT->get_e_scale_fnc = nullptr;
}

void Extruder::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(extruder_checksum)) return;

    if(!this->selected) return;

    // pointer to structure to return data to is provided
    pad_extruder_t *e = static_cast<pad_extruder_t *>(pdr->get_data_ptr());
    e->steps_per_mm = stepper_motor->get_steps_per_mm();
    e->filament_diameter = this->filament_diameter;
    e->flow_rate = this->extruder_multiplier;
    e->accleration = stepper_motor->get_acceleration();
    e->retract_length = this->retract_length;
    e->current_position = stepper_motor->get_current_position();
    pdr->set_taken();
}

void Extruder::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(extruder_checksum)) return;

    // handle extrude rates request from robot
    if(pdr->second_element_is(target_checksum)) {
        // disabled extruders do not reply NOTE only one enabled extruder supported
        if(!this->selected) return;

        float *d = static_cast<float *>(pdr->get_data_ptr());
        float delta = d[0]; // the E passed in on Gcode is the delta volume in mm³
        float isecs = d[1]; // inverted secs

        // check against maximum speeds and return rate modifier
        d[1] = check_max_speeds(delta, isecs);
        pdr->set_taken();
        return;
    }

    // save or restore extruder state
    if(pdr->second_element_is(save_state_checksum)) {
        save_position();
        this->saved_selected = this->selected;
        pdr->set_taken();

    } else if(pdr->second_element_is(restore_state_checksum)) {
        this->selected = this->saved_selected;
        // NOTE this only gets called when the queue is empty so the milestones will be the same
        restore_position();
        pdr->set_taken();
    }
}

void Extruder::save_position()
{
    // we need to save these separately as they may have been scaled
    this->saved_position = std::make_tuple(THEROBOT->get_axis_position(motor_id), stepper_motor->get_last_milestone(), stepper_motor->get_last_milestone_steps());
}

void Extruder::restore_position()
{
    THEROBOT->reset_axis_position(std::get<0>(this->saved_position), motor_id);
    stepper_motor->set_last_milestones(std::get<1>(this->saved_position), std::get<2>(this->saved_position));
}

// check against maximum speeds and return the rate modifier
float Extruder::check_max_speeds(float delta, float isecs)
{
    float rm = 1.0F; // default no rate modification

    if(this->max_volumetric_rate > 0 && this->filament_diameter > 0.01F) {
        // volumetric enabled and check for volumetric rate
        float v = delta * isecs; // the flow rate in mm³/sec

        // return the rate change needed to stay within the max rate
        if(v > max_volumetric_rate) {
            rm = max_volumetric_rate / v;
        }
        //THEKERNEL->streams->printf("requested flow rate: %f mm³/sec, corrected flow rate: %f  mm³/sec\n", v, v * rm);
    }

    return rm;
}

void Extruder::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes most execute immediately, most only execute if enabled
    if (gcode->has_m) {
        if (gcode->m == 114 && this->selected) {
            char buf[16];
            if(gcode->subcode == 0) {
                float pos = THEROBOT->get_axis_position(motor_id);
                int n = snprintf(buf, sizeof(buf), " E:%1.3f ", pos);
                gcode->txt_after_ok.append(buf, n);

            } else if(gcode->subcode == 1) { // realtime position
                int n = snprintf(buf, sizeof(buf), " E:%1.3f ", stepper_motor->get_current_position() / get_e_scale());
                gcode->txt_after_ok.append(buf, n);

            } else if(gcode->subcode == 3) { // realtime actuator position
                int n = snprintf(buf, sizeof(buf), " E:%1.3f ", stepper_motor->get_current_position());
                gcode->txt_after_ok.append(buf, n);
            }


        } else if (gcode->m == 92 && ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier) ) ) {
            float spm = stepper_motor->get_steps_per_mm();
            if (gcode->has_letter('E')) {
                spm = gcode->get_value('E');
                stepper_motor->change_steps_per_mm(spm);
            }

            gcode->stream->printf("E:%f ", spm);
            gcode->add_nl = true;

        } else if (gcode->m == 200 && ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            if (gcode->has_letter('D')) {
                this->filament_diameter = gcode->get_value('D');
                float last_scale = this->volumetric_multiplier;
                if(filament_diameter > 0.01F) {
                    this->volumetric_multiplier = 1.0F / (powf(this->filament_diameter / 2, 2) * PI);
                } else {
                    this->volumetric_multiplier = 1.0F;
                }
                // the trouble here is that the last milestone will be for the previous multiplier so a change can cause a big blob
                // so we must change the E last milestone accordingly so it continues smoothly....
                // change E last milestone to what it would have been if it had used this new multiplier
                float delta = this->volumetric_multiplier / last_scale;
                float nm = this->stepper_motor->get_last_milestone() * delta;
                this->stepper_motor->change_last_milestone(nm);

            } else {
                if(filament_diameter > 0.01F) {
                    gcode->stream->printf("Filament Diameter: %f\n", this->filament_diameter);
                } else {
                    gcode->stream->printf("Volumetric extrusion is disabled\n");
                }
            }

        } else if (gcode->m == 203 && ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // M203 Exxx Vyyy Set maximum feedrates xxx mm/sec and/or yyy mm³/sec
            if(gcode->get_num_args() == 0) {
                gcode->stream->printf("E:%g V:%g", this->stepper_motor->get_max_rate(), this->max_volumetric_rate);
                gcode->add_nl = true;

            } else {
                if(gcode->has_letter('E')) {
                    this->stepper_motor->set_max_rate(gcode->get_value('E'));
                }
                if(gcode->has_letter('V')) {
                    this->max_volumetric_rate = gcode->get_value('V');
                }
            }

        } else if (gcode->m == 204 && gcode->has_letter('E') &&
                   ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // extruder acceleration M204 Ennn mm/sec^2 (Pnnn sets the specific extruder for M500)
            stepper_motor->set_acceleration(gcode->get_value('E'));

        } else if (gcode->m == 207 && ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop] Q[zlift feedrate mm/min]
            if(gcode->has_letter('S')) retract_length = gcode->get_value('S');
            if(gcode->has_letter('F')) retract_feedrate = gcode->get_value('F') / 60.0F; // specified in mm/min converted to mm/sec
            if(gcode->has_letter('Z')) retract_zlift_length = gcode->get_value('Z'); // specified in mm
            if(gcode->has_letter('Q')) retract_zlift_feedrate = gcode->get_value('Q') / 60.0F; // specified in mm/min converted to mm/sec

        } else if (gcode->m == 208 && ( (this->selected && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
            if(gcode->has_letter('S')) retract_recover_length = gcode->get_value('S');
            if(gcode->has_letter('F')) retract_recover_feedrate = gcode->get_value('F') / 60.0F; // specified in mm/min converted to mm/sec

        } else if (gcode->m == 221 && this->selected) { // M221 S100 change flow rate by percentage
            if(gcode->has_letter('S')) {
                float last_scale = this->extruder_multiplier;
                this->extruder_multiplier = gcode->get_value('S') / 100.0F;
                // the trouble here is that the last milestone will be for the previous multiplier so a change can cause a big blob
                // so we must change the E last milestone accordingly so it continues smoothly....
                // change E last milestone to what it would have been if it had used this new multiplier
                float delta = this->extruder_multiplier / last_scale;
                float nm = this->stepper_motor->get_last_milestone() * delta;
                this->stepper_motor->change_last_milestone(nm);

            } else {
                gcode->stream->printf("Flow rate at %6.2f %%\n", this->extruder_multiplier * 100.0F);
            }

        } else if (gcode->m == 500 || gcode->m == 503) { // M500 saves some volatile settings to config override file, M503 just prints the settings
            gcode->stream->printf(";E Steps per mm:\nM92 E%1.4f P%d\n", stepper_motor->get_steps_per_mm(), this->identifier);
            gcode->stream->printf(";E Filament diameter:\nM200 D%1.4f P%d\n", this->filament_diameter, this->identifier);
            gcode->stream->printf(";E retract length, feedrate:\nM207 S%1.4f F%1.4f Z%1.4f Q%1.4f P%d\n", this->retract_length, this->retract_feedrate * 60.0F, this->retract_zlift_length, this->retract_zlift_feedrate * 60.0F, this->identifier);
            gcode->stream->printf(";E retract recover length, feedrate:\nM208 S%1.4f F%1.4f P%d\n", this->retract_recover_length, this->retract_recover_feedrate * 60.0F, this->identifier);
            gcode->stream->printf(";E acceleration mm/sec²:\nM204 E%1.4f P%d\n", stepper_motor->get_acceleration(), this->identifier);
            gcode->stream->printf(";E max feed rate mm/sec:\nM203 E%1.4f P%d\n", stepper_motor->get_max_rate(), this->identifier);
            if(this->max_volumetric_rate > 0) {
                gcode->stream->printf(";E max volumetric rate mm³/sec:\nM203 V%1.4f P%d\n", this->max_volumetric_rate, this->identifier);
            }
        }

    } else if( gcode->has_g && this->selected ) {

        if( (gcode->g == 10 || gcode->g == 11) && !gcode->has_letter('L') ) {
            // firmware retract command (Ignore if has L parameter that is not for us)
            // check we are in the correct state of retract or unretract
            if(gcode->g == 10 && !retracted) {
                this->retracted = true;
                this->cancel_zlift_restore = false;
                this->g92e0_detected = false;
            } else if(gcode->g == 11 && retracted) {
                this->retracted = false;
            } else
                return; // ignore duplicates

            if(gcode->g == 10) {
                // retract
                float delta[motor_id + 1];
                for (int i = 0; i < motor_id; ++i) {
                    delta[i] = 0;
                }

                delta[motor_id] = -retract_length / get_e_scale(); // convert from mm to mm³, and unapply flow_rate
                THEROBOT->delta_move(delta, retract_feedrate, motor_id + 1);

                // zlift
                if(retract_zlift_length > 0) {
                    float delta[3] {0, 0, retract_zlift_length};
                    THEROBOT->delta_move(delta, retract_zlift_feedrate, 3);
                }

            } else if(gcode->g == 11) {
                // unretract
                if(retract_zlift_length > 0 && !this->cancel_zlift_restore) {
                    // reverse zlift happens before unretract
                    // NOTE we do not do this if cancel_zlift_restore is set to true, which happens if there is an absolute Z move inbetween G10 and G11
                    float delta[3] {0, 0, -retract_zlift_length};
                    THEROBOT->delta_move(delta, retract_zlift_feedrate, 3);
                }

                float delta[motor_id + 1];
                for (int i = 0; i < motor_id; ++i) {
                    delta[i] = 0;
                }
                // HACK ALERT due to certain slicers reseting E with G92 E0 between the G10 and G11 we need to restore
                // the current position after we do the unretract, this is horribly hacky :(
                // also as the move has not completed yet, when we restore the current position will be incorrect once the move finishes,
                // however this is not fatal for an extruder
                if(g92e0_detected) save_position();
                delta[motor_id] = (retract_length + retract_recover_length) / get_e_scale(); // convert from mm to mm³, and unapply flow_rate
                THEROBOT->delta_move(delta, retract_recover_feedrate, motor_id + 1);
                if(g92e0_detected) restore_position();
            }


        } else if( this->retracted && (gcode->g == 0 || gcode->g == 1) && gcode->has_letter('Z')) {
            // NOTE we cancel the zlift restore for the following G11 as we have moved to an absolute Z which we need to stay at
            this->cancel_zlift_restore = true;

        } else if( this->retracted && gcode->g == 92 && gcode->has_letter('E')) {
            // old versions of slic3rs issued a G92 E0 after G10, handle that case
            this->g92e0_detected= true;
        }

    }

}
