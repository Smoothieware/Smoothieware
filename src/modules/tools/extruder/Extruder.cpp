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
#include "Stepper.h"
#include "StepTicker.h"
#include "Config.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"

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

// NEW config names
#define extruder_checksum                    CHECKSUM("extruder")

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

#define X_AXIS      0
#define Y_AXIS      1
#define Z_AXIS      2

#define OFF 0
#define SOLO 1
#define FOLLOW 2

#define PI 3.14159265358979F

#define max(a,b) (((a) > (b)) ? (a) : (b))

/* The extruder module controls a filament extruder for 3D printing: http://en.wikipedia.org/wiki/Fused_deposition_modeling
* It can work in two modes : either the head does not move, and the extruder moves the filament at a specified speed ( SOLO mode here )
* or the head moves, and the extruder moves plastic at a speed proportional to the movement of the head ( FOLLOW mode here ).
*/

Extruder::Extruder( uint16_t config_identifier, bool single )
{
    this->absolute_mode = true;
    this->enabled       = false;
    this->paused        = false;
    this->single_config = single;
    this->identifier    = config_identifier;
    this->retracted     = false;
    this->volumetric_multiplier= 1.0F;

    memset(this->offset, 0, sizeof(this->offset));
}

void Extruder::on_module_loaded()
{

    // Settings
    this->on_config_reload(this);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_GET_PUBLIC_DATA);

    // Start values
    this->target_position = 0;
    this->current_position = 0;
    this->unstepped_distance = 0;
    this->current_block = NULL;
    this->mode = OFF;

    // Update speed every *acceleration_ticks_per_second*
    // TODO: Make this an independent setting
    THEKERNEL->slow_ticker->attach( THEKERNEL->stepper->get_acceleration_ticks_per_second() , this, &Extruder::acceleration_tick );

    // Stepper motor object for the extruder
    this->stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(step_pin, dir_pin, en_pin) );
    this->stepper_motor->attach(this, &Extruder::stepper_motor_finished_move );
}

// Get config
void Extruder::on_config_reload(void *argument)
{
    if( this->single_config ) {
        // If this module uses the old "single extruder" configuration style

        this->steps_per_millimeter        = THEKERNEL->config->value(extruder_steps_per_mm_checksum      )->by_default(1)->as_number();
        this->filament_diameter           = THEKERNEL->config->value(extruder_filament_diameter_checksum )->by_default(0)->as_number();
        this->acceleration                = THEKERNEL->config->value(extruder_acceleration_checksum      )->by_default(1000)->as_number();
        this->max_speed                   = THEKERNEL->config->value(extruder_max_speed_checksum         )->by_default(1000)->as_number();
        this->feed_rate                   = THEKERNEL->config->value(default_feed_rate_checksum          )->by_default(1000)->as_number();

        this->step_pin.from_string(         THEKERNEL->config->value(extruder_step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
        this->dir_pin.from_string(          THEKERNEL->config->value(extruder_dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
        this->en_pin.from_string(           THEKERNEL->config->value(extruder_en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

        for(int i = 0; i < 3; i++) {
            this->offset[i] = 0;
        }

        this->enabled = true;

    } else {
        // If this module was created with the new multi extruder configuration style

        this->steps_per_millimeter = THEKERNEL->config->value(extruder_checksum, this->identifier, steps_per_mm_checksum      )->by_default(1)->as_number();
        this->filament_diameter    = THEKERNEL->config->value(extruder_checksum, this->identifier, filament_diameter_checksum )->by_default(0)->as_number();
        this->acceleration         = THEKERNEL->config->value(extruder_checksum, this->identifier, acceleration_checksum      )->by_default(1000)->as_number();
        this->max_speed            = THEKERNEL->config->value(extruder_checksum, this->identifier, max_speed_checksum         )->by_default(1000)->as_number();
        this->feed_rate            = THEKERNEL->config->value(                                     default_feed_rate_checksum )->by_default(1000)->as_number();

        this->step_pin.from_string( THEKERNEL->config->value(extruder_checksum, this->identifier, step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
        this->dir_pin.from_string(  THEKERNEL->config->value(extruder_checksum, this->identifier, dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
        this->en_pin.from_string(   THEKERNEL->config->value(extruder_checksum, this->identifier, en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

        this->offset[X_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, x_offset_checksum          )->by_default(0)->as_number();
        this->offset[Y_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, y_offset_checksum          )->by_default(0)->as_number();
        this->offset[Z_AXIS] = THEKERNEL->config->value(extruder_checksum, this->identifier, z_offset_checksum          )->by_default(0)->as_number();

    }

    // these are only supported in the new syntax, no need to be backward compatible as they did not exist before the change
    this->retract_length           = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_length_checksum)->by_default(3)->as_number();
    this->retract_feedrate         = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_feedrate_checksum)->by_default(45)->as_number();
    this->retract_recover_length   = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_recover_length_checksum)->by_default(0)->as_number();
    this->retract_recover_feedrate = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_recover_feedrate_checksum)->by_default(8)->as_number();
    this->retract_zlift_length     = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_zlift_length_checksum)->by_default(0)->as_number();
    this->retract_zlift_feedrate   = THEKERNEL->config->value(extruder_checksum, this->identifier, retract_zlift_feedrate_checksum)->by_default(100*60)->as_number(); // mm/min

    if(filament_diameter > 0.01) {
        this->volumetric_multiplier = 1.0F / (powf(this->filament_diameter / 2, 2) * PI);
    }
}

void Extruder::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(extruder_checksum)) return;

    if(this->enabled) {
        // Note this is allowing both step/mm and filament diameter to be exposed via public data
        pdr->set_data_ptr(&this->steps_per_millimeter);
        pdr->set_taken();
    }
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Extruder::on_pause(void *argument)
{
    this->paused = true;
    this->stepper_motor->pause();
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Extruder::on_play(void *argument)
{
    this->paused = false;
    this->stepper_motor->unpause();
}

void Extruder::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    // M codes most execute immediately, most only execute if enabled
    if (gcode->has_m) {
        if (gcode->m == 114 && this->enabled) {
            char buf[16];
            int n = snprintf(buf, sizeof(buf), " E:%1.3f ", this->current_position);
            gcode->txt_after_ok.append(buf, n);
            gcode->mark_as_taken();

        } else if (gcode->m == 92 && ( (this->enabled && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier) ) ) {
            float spm = this->steps_per_millimeter;
            if (gcode->has_letter('E')) {
                spm = gcode->get_value('E');
                this->steps_per_millimeter = spm;
            }

            gcode->stream->printf("E:%g ", spm);
            gcode->add_nl = true;
            gcode->mark_as_taken();

        } else if (gcode->m == 200 && ( (this->enabled && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            if (gcode->has_letter('D')) {
                THEKERNEL->conveyor->wait_for_empty_queue(); // only apply after the queue has emptied
                this->filament_diameter = gcode->get_value('D');
                if(filament_diameter > 0.01) {
                    this->volumetric_multiplier = 1.0F / (powf(this->filament_diameter / 2, 2) * PI);
                }else{
                    this->volumetric_multiplier = 1.0F;
                }
            }
            gcode->mark_as_taken();

        } else if (gcode->m == 207 && ( (this->enabled && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop] Q[zlift feedrate mm/min]
            if(gcode->has_letter('S')) retract_length = gcode->get_value('S');
            if(gcode->has_letter('F')) retract_feedrate = gcode->get_value('F')/60.0F; // specified in mm/min converted to mm/sec
            if(gcode->has_letter('Z')) retract_zlift_length = gcode->get_value('Z');
            if(gcode->has_letter('Q')) retract_zlift_feedrate = gcode->get_value('Q');
            gcode->mark_as_taken();

        } else if (gcode->m == 208 && ( (this->enabled && !gcode->has_letter('P')) || (gcode->has_letter('P') && gcode->get_value('P') == this->identifier)) ) {
            // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
            if(gcode->has_letter('S')) retract_recover_length = gcode->get_value('S');
            if(gcode->has_letter('F')) retract_recover_feedrate = gcode->get_value('F')/60.0F; // specified in mm/min converted to mm/sec
            gcode->mark_as_taken();

        } else if (gcode->m == 500 || gcode->m == 503) { // M500 saves some volatile settings to config override file, M503 just prints the settings
            if( this->single_config ) {
                gcode->stream->printf(";E Steps per mm:\nM92 E%1.4f\n", this->steps_per_millimeter);
                gcode->stream->printf(";E Filament diameter:\nM200 D%1.4f\n", this->filament_diameter);
                gcode->stream->printf(";E retract length, feedrate, zlift length, feedrate:\nM207 S%1.4f F%1.4f Z%1.4f Q%1.4f\n", this->retract_length, this->retract_feedrate*60.0F, this->retract_zlift_length, this->retract_zlift_feedrate);
                gcode->stream->printf(";E retract recover length, feedrate:\nM208 S%1.4f F%1.4f\n", this->retract_recover_length, this->retract_recover_feedrate*60.0F);

            } else {
                gcode->stream->printf(";E Steps per mm:\nM92 E%1.4f P%d\n", this->steps_per_millimeter, this->identifier);
                gcode->stream->printf(";E Filament diameter:\nM200 D%1.4f P%d\n", this->filament_diameter, this->identifier);
                gcode->stream->printf(";E retract length, feedrate:\nM207 S%1.4f F%1.4f Z%1.4f Q%1.4f P%d\n", this->retract_length, this->retract_feedrate*60.0F, this->retract_zlift_length, this->retract_zlift_feedrate, this->identifier);
                gcode->stream->printf(";E retract recover length, feedrate:\nM208 S%1.4f F%1.4f P%d\n", this->retract_recover_length, this->retract_recover_feedrate*60.0F, this->identifier);
            }
            gcode->mark_as_taken();

        } else if( gcode->m == 17 || gcode->m == 18 || gcode->m == 82 || gcode->m == 83 || gcode->m == 84 ) {
            // Mcodes to pass along to on_gcode_execute
            THEKERNEL->conveyor->append_gcode(gcode);
            gcode->mark_as_taken();
        }

    }else if(gcode->has_g) {
        // G codes, NOTE some are ignored if not enabled
        if( (gcode->g == 92 && gcode->has_letter('E')) || (gcode->g == 90 || gcode->g == 91) ) {
            // Gcodes to pass along to on_gcode_execute
            THEKERNEL->conveyor->append_gcode(gcode);
            gcode->mark_as_taken();

        }else if( this->enabled && gcode->g < 4 && gcode->has_letter('E') && !gcode->has_letter('X') && !gcode->has_letter('Y') && !gcode->has_letter('Z') ) {
            // This is a solo move, we add an empty block to the queue to prevent subsequent gcodes being executed at the same time
            THEKERNEL->conveyor->append_gcode(gcode);
            THEKERNEL->conveyor->queue_head_block();
            gcode->mark_as_taken();

        }else if( this->enabled && (gcode->g == 10 || gcode->g == 11) ) { // firmware retract command
            gcode->mark_as_taken();
            // check we are in the correct state of retract or unretract
            if(gcode->g == 10 && !retracted)
                retracted= true;
            else if(gcode->g == 11 && retracted)
                retracted= false;
            else
                return; // ignore duplicates

            // now we do a special hack to add zlift if needed, this should go in Robot but if it did the zlift would be executed before retract which is bad
            // this way zlift will happen after retract, (or before for unretract) NOTE we call the robot->on_gcode_receive directly to avoid recursion
            if(retract_zlift_length > 0 && gcode->g == 11) {
                // reverse zlift happens before unretract
                char buf[32];
                int n= snprintf(buf, sizeof(buf), "G0 Z%1.4f F%1.4f", -retract_zlift_length, retract_zlift_feedrate);
                string cmd(buf, n);
                Gcode gc(cmd, &(StreamOutput::NullStream));
                bool oldmode= THEKERNEL->robot->absolute_mode;
                THEKERNEL->robot->absolute_mode= false; // needs to be relative mode
                THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
                THEKERNEL->robot->absolute_mode= oldmode; // restore mode
            }

            // This is a solo move, we add an empty block to the queue to prevent subsequent gcodes being executed at the same time
            THEKERNEL->conveyor->append_gcode(gcode);
            THEKERNEL->conveyor->queue_head_block();

            if(retract_zlift_length > 0 && gcode->g == 10) {
                char buf[32];
                int n= snprintf(buf, sizeof(buf), "G0 Z%1.4f F%1.4f", retract_zlift_length, retract_zlift_feedrate);
                string cmd(buf, n);
                Gcode gc(cmd, &(StreamOutput::NullStream));
                bool oldmode= THEKERNEL->robot->absolute_mode;
                THEKERNEL->robot->absolute_mode= false; // needs to be relative mode
                THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
                THEKERNEL->robot->absolute_mode= oldmode; // restore mode
            }
        }
    }
}

// Compute extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    // The mode is OFF by default, and SOLO or FOLLOW only if we need to extrude
    this->mode = OFF;

    // Absolute/relative mode, globably modal affect all extruders whether enabled or not
    if( gcode->has_m ) {
        switch(gcode->m) {
        case 17:
            this->en_pin.set(0);
            break;
        case 18:
            this->en_pin.set(1);
            break;
        case 82:
            this->absolute_mode = true;
            break;
        case 83:
            this->absolute_mode = false;
            break;
        case 84:
            this->en_pin.set(1);
            break;
        }
        return;

    } else if( gcode->has_g && (gcode->g == 90 || gcode->g == 91) ) {
        this->absolute_mode = (gcode->g == 90);
        return;
    }


    if( gcode->has_g && this->enabled ) {
        // G92: Reset extruder position
        if( gcode->g == 92 ) {
            if( gcode->has_letter('E') ) {
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->unstepped_distance = 0;
            } else if( gcode->get_num_args() == 0) {
                this->current_position = 0.0;
                this->target_position = this->current_position;
                this->unstepped_distance = 0;
            }

        } else if (gcode->g == 10) {
            // FW retract command
            feed_rate= retract_feedrate; // mm/sec
            this->mode = SOLO;
            this->travel_distance = -retract_length;
            this->target_position += this->travel_distance;
            this->en_pin.set(0);

        } else if (gcode->g == 11) {
            // un retract command
            feed_rate= retract_recover_feedrate; // mm/sec
            this->mode = SOLO;
            this->travel_distance = (retract_length + retract_recover_length);
            this->target_position += this->travel_distance;
            this->en_pin.set(0);

        } else if (gcode->g == 0 || gcode->g == 1) {
            // Extrusion length from 'G' Gcode
            if( gcode->has_letter('E' )) {
                // Get relative extrusion distance depending on mode ( in absolute mode we must substract target_position )
                float extrusion_distance = gcode->get_value('E');
                float relative_extrusion_distance = extrusion_distance;
                if (this->absolute_mode) {
                    relative_extrusion_distance -= this->target_position;
                    this->target_position = extrusion_distance;
                } else {
                    this->target_position += relative_extrusion_distance;
                }

                // If the robot is moving, we follow it's movement, otherwise, we move alone
                if( fabs(gcode->millimeters_of_travel) < 0.0001F ) { // With floating numbers, we can have 0 != 0 ... beeeh. For more info see : http://upload.wikimedia.org/wikipedia/commons/0/0a/Cain_Henri_Vidal_Tuileries.jpg
                    this->mode = SOLO;
                    this->travel_distance = relative_extrusion_distance;
                } else {
                    // We move proportionally to the robot's movement
                    this->mode = FOLLOW;
                    this->travel_ratio = (relative_extrusion_distance * this->volumetric_multiplier) / gcode->millimeters_of_travel; // adjust for volumetric extrusion
                    // TODO: check resulting flowrate, limit robot speed if it exceeds max_speed
                }

                this->en_pin.set(0);
            }

            if (gcode->has_letter('F')) {
                feed_rate = gcode->get_value('F') / THEKERNEL->robot->get_seconds_per_minute();
                if (feed_rate > max_speed)
                    feed_rate = max_speed;
            }
        }
    }

}

// When a new block begins, either follow the robot, or step by ourselves ( or stay back and do nothing )
void Extruder::on_block_begin(void *argument)
{
    if(!this->enabled) return;
    Block *block = static_cast<Block *>(argument);


    if( this->mode == SOLO ) {
        // In solo mode we take the block so we can move even if the stepper has nothing to do

        this->current_position += this->travel_distance ;

        int steps_to_step = abs(int(floor(this->steps_per_millimeter * (this->travel_distance + this->unstepped_distance) )));

        if ( this->travel_distance > 0 ) {
            this->unstepped_distance += this->travel_distance - (steps_to_step / this->steps_per_millimeter); //catch any overflow
        }   else {
            this->unstepped_distance += this->travel_distance + (steps_to_step / this->steps_per_millimeter); //catch any overflow
        }

        if( steps_to_step != 0 ) {

            // We take the block, we have to release it or everything gets stuck
            block->take();
            this->current_block = block;

            this->stepper_motor->set_steps_per_second(0);
            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step);

        } else {
            this->current_block = NULL;
        }

    } else if( this->mode == FOLLOW ) {
        // In non-solo mode, we just follow the stepper module
        this->travel_distance = block->millimeters * this->travel_ratio;

        this->current_position += this->travel_distance;

        int steps_to_step = abs(int(floor(this->steps_per_millimeter * (this->travel_distance + this->unstepped_distance) )));

        if ( this->travel_distance > 0 ) {
            this->unstepped_distance += this->travel_distance - (steps_to_step / this->steps_per_millimeter); //catch any overflow
        }   else {
            this->unstepped_distance += this->travel_distance + (steps_to_step / this->steps_per_millimeter); //catch any overflow
        }

        if( steps_to_step != 0 ) {
            block->take();
            this->current_block = block;

            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step );
            this->on_speed_change(0); // initialise speed in case we get called first
        } else {
            this->current_block = NULL;
        }

    } else if( this->mode == OFF ) {
        // No movement means we must reset our speed
        this->current_block = NULL;
        //this->stepper_motor->set_speed(0);

    }

}

// When a block ends, pause the stepping interrupt
void Extruder::on_block_end(void *argument)
{
    if(!this->enabled) return;
    this->current_block = NULL;
}

// Called periodically to change the speed to match acceleration or to match the speed of the robot
uint32_t Extruder::acceleration_tick(uint32_t dummy)
{
    if(!this->enabled) return 0;

    // Avoid trying to work when we really shouldn't ( between blocks or re-entry )
    if( this->current_block == NULL ||  this->paused || this->mode != SOLO ) {
        return 0;
    }

    uint32_t current_rate = this->stepper_motor->get_steps_per_second();
    uint32_t target_rate = int(floor(this->feed_rate * this->steps_per_millimeter));

    if( current_rate < target_rate ) {
        uint32_t rate_increase = int(floor((this->acceleration / THEKERNEL->stepper->get_acceleration_ticks_per_second()) * this->steps_per_millimeter));
        current_rate = min( target_rate, current_rate + rate_increase );
    }
    if( current_rate > target_rate ) {
        current_rate = target_rate;
    }

    // steps per second
    this->stepper_motor->set_speed(max(current_rate, THEKERNEL->stepper->get_minimum_steps_per_second()));

    return 0;
}

// Speed has been updated for the robot's stepper, we must update accordingly
void Extruder::on_speed_change( void *argument )
{
    if(!this->enabled) return;

    // Avoid trying to work when we really shouldn't ( between blocks or re-entry )
    if( this->current_block == NULL ||  this->paused || this->mode != FOLLOW || this->stepper_motor->is_moving() != true ) {
        return;
    }

    /*
    * nominal block duration = current block's steps / ( current block's nominal rate )
    * nominal extruder rate = extruder steps / nominal block duration
    * actual extruder rate = nominal extruder rate * ( ( stepper's steps per second ) / ( current block's nominal rate ) )
    * or actual extruder rate = ( ( extruder steps * ( current block's nominal_rate ) ) / current block's steps ) * ( ( stepper's steps per second ) / ( current block's nominal rate ) )
    * or simplified : extruder steps * ( stepper's steps per second ) ) / current block's steps
    * or even : ( stepper steps per second ) * ( extruder steps / current block's steps )
    */

    this->stepper_motor->set_speed( max( ( THEKERNEL->stepper->get_trapezoid_adjusted_rate()) * ( (float)this->stepper_motor->get_steps_to_move() / (float)this->current_block->steps_event_count ), THEKERNEL->stepper->get_minimum_steps_per_second() ) );

}

// When the stepper has finished it's move
uint32_t Extruder::stepper_motor_finished_move(uint32_t dummy)
{
    if(!this->enabled) return 0;

    //printf("extruder releasing\r\n");

    if (this->current_block) { // this should always be true, but sometimes it isn't. TODO: find out why
        Block *block = this->current_block;
        this->current_block = NULL;
        block->release();
    }
    return 0;

}
