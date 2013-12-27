/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/Block.h"
#include "modules/tools/extruder/Extruder.h"
#include <mri.h>

#define extruder_module_enable_checksum      CHECKSUM("extruder_module_enable")
#define extruder_steps_per_mm_checksum       CHECKSUM("extruder_steps_per_mm")
#define extruder_acceleration_checksum       CHECKSUM("extruder_acceleration")
#define extruder_step_pin_checksum           CHECKSUM("extruder_step_pin")
#define extruder_dir_pin_checksum            CHECKSUM("extruder_dir_pin")
#define extruder_en_pin_checksum             CHECKSUM("extruder_en_pin")
#define extruder_max_speed_checksum          CHECKSUM("extruder_max_speed")

#define extruder_checksum                    CHECKSUM("extruder")

#define default_feed_rate_checksum           CHECKSUM("default_feed_rate")
#define steps_per_mm_checksum                CHECKSUM("steps_per_mm")
#define acceleration_checksum                CHECKSUM("acceleration")
#define step_pin_checksum                    CHECKSUM("step_pin")
#define dir_pin_checksum                     CHECKSUM("dir_pin")
#define en_pin_checksum                      CHECKSUM("en_pin")
#define max_speed_checksum                   CHECKSUM("max_speed")

#define max(a,b) (((a) > (b)) ? (a) : (b))

/* The extruder module controls a filament extruder for 3D printing: http://en.wikipedia.org/wiki/Fused_deposition_modeling
* It can work in two modes : either the head does not move, and the extruder moves the filament at a specified speed ( SOLO mode here )
* or the head moves, and the extruder moves plastic at a speed proportional to the movement of the head ( FOLLOW mode here ).
*/

Extruder::Extruder( uint16_t config_identifier ) {
    this->absolute_mode = true;
    this->paused        = false;
    this->single_config = false;
    this->identifier    = config_identifier;
}

void Extruder::on_module_loaded() {

    // Settings
    this->on_config_reload(this);

    // We start with the enable pin off
    this->en_pin.set(1);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_SPEED_CHANGE);

    // Start values
    this->target_position = 0;
    this->current_position = 0;
    this->unstepped_distance = 0;
    this->current_block = NULL;
    this->mode = OFF;

    // Update speed every *acceleration_ticks_per_second*
    // TODO: Make this an independent setting
    THEKERNEL->slow_ticker->attach( THEKERNEL->stepper->acceleration_ticks_per_second , this, &Extruder::acceleration_tick );

    // Stepper motor object for the extruder
    this->stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(&step_pin, &dir_pin, &en_pin) );
    this->stepper_motor->attach(this, &Extruder::stepper_motor_finished_move );

}

// Get config
void Extruder::on_config_reload(void* argument){

    // If this module uses the old "single extruder" configuration style
    if( this->single_config ){

        this->steps_per_millimeter        = THEKERNEL->config->value(extruder_steps_per_mm_checksum      )->by_default(1)->as_number();
        this->acceleration                = THEKERNEL->config->value(extruder_acceleration_checksum      )->by_default(1000)->as_number();
        this->max_speed                   = THEKERNEL->config->value(extruder_max_speed_checksum         )->by_default(1000)->as_number();
        this->feed_rate                   = THEKERNEL->config->value(default_feed_rate_checksum          )->by_default(1000)->as_number();

        this->step_pin.from_string(         THEKERNEL->config->value(extruder_step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
        this->dir_pin.from_string(          THEKERNEL->config->value(extruder_dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
        this->en_pin.from_string(           THEKERNEL->config->value(extruder_en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

    }else{
    // If this module was created with the new multi extruder configuration style

        this->steps_per_millimeter        = THEKERNEL->config->value(extruder_checksum, this->identifier, steps_per_mm_checksum      )->by_default(1)->as_number();
        this->acceleration                = THEKERNEL->config->value(extruder_checksum, this->identifier, acceleration_checksum      )->by_default(1000)->as_number();
        this->max_speed                   = THEKERNEL->config->value(extruder_checksum, this->identifier, max_speed_checksum         )->by_default(1000)->as_number();
        this->feed_rate                   = THEKERNEL->config->value(                                     default_feed_rate_checksum )->by_default(1000)->as_number();

        this->step_pin.from_string(         THEKERNEL->config->value(extruder_checksum, this->identifier, step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
        this->dir_pin.from_string(          THEKERNEL->config->value(extruder_checksum, this->identifier, dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
        this->en_pin.from_string(           THEKERNEL->config->value(extruder_checksum, this->identifier, en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

    }

    // disable by default
    this->en_pin.set(1);

}


// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Extruder::on_pause(void* argument){
    this->paused = true;
    this->stepper_motor->pause();
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Extruder::on_play(void* argument){
    this->paused = false;
    this->stepper_motor->unpause();
}


void Extruder::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode*>(argument);

    // Gcodes to execute immediately
    if (gcode->has_m){
        if (gcode->m == 114){
            gcode->stream->printf("E:%4.1f ", this->current_position);
            gcode->add_nl = true;
            gcode->mark_as_taken();

        }else if (gcode->m == 92 ){
            float spm = this->steps_per_millimeter;
            if (gcode->has_letter('E'))
                spm = gcode->get_value('E');
            gcode->stream->printf("E:%g ", spm);
            gcode->add_nl = true;
            gcode->mark_as_taken();

        }else if (gcode->m == 500 || gcode->m == 503){// M500 saves some volatile settings to config override file, M503 just prints the settings
            gcode->stream->printf(";E Steps per mm:\nM92 E%1.4f\n", this->steps_per_millimeter);
            gcode->mark_as_taken();
            return;
        }
    }

    // Gcodes to pass along to on_gcode_execute
    if( ( gcode->has_m && (gcode->m == 17 || gcode->m == 18 || gcode->m == 82 || gcode->m == 83 || gcode->m == 84 || gcode->m == 92 ) ) || ( gcode->has_g && gcode->g == 92 && gcode->has_letter('E') ) || ( gcode->has_g && ( gcode->g == 90 || gcode->g == 91 ) ) ){
        gcode->mark_as_taken();
        if( THEKERNEL->conveyor->queue.size() == 0 ){
            THEKERNEL->call_event(ON_GCODE_EXECUTE, gcode );
        }else{
            Block* block = THEKERNEL->conveyor->queue.get_ref( THEKERNEL->conveyor->queue.size() - 1 );
            block->append_gcode(gcode);
        }
    }

    // Add to the queue for on_gcode_execute to process
    if( gcode->has_g && gcode->g < 4 && gcode->has_letter('E') ){
        if( !gcode->has_letter('X') && !gcode->has_letter('Y') && !gcode->has_letter('Z') ){
            // This is a solo move, we add an empty block to the queue
            //If the queue is empty, execute immediatly, otherwise attach to the last added block
            if( THEKERNEL->conveyor->queue.size() == 0 ){
                THEKERNEL->call_event(ON_GCODE_EXECUTE, gcode );
                this->append_empty_block();
            }else{
                Block* block = THEKERNEL->conveyor->queue.get_ref( THEKERNEL->conveyor->queue.size() - 1 );
                block->append_gcode(gcode);
                this->append_empty_block();
            }
        }
    }else{
        // This is for follow move

    }
}

// Append an empty block in the queue so that solo mode can pick it up
Block* Extruder::append_empty_block(){
    THEKERNEL->conveyor->wait_for_queue(2);
    Block* block = THEKERNEL->conveyor->new_block();
    block->planner = THEKERNEL->planner;
    block->millimeters = 0;
    block->steps[0] = 0;
    block->steps[1] = 0;
    block->steps[2] = 0;
    // feed the block into the system. Will execute it if we are at the beginning of the queue
    block->ready();

    return block;
}

// Compute extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    // Absolute/relative mode
    if( gcode->has_m ){
        if( gcode->m == 17 ){ this->en_pin.set(0); }
        if( gcode->m == 18 ){ this->en_pin.set(1); }
        if( gcode->m == 82 ){ this->absolute_mode = true; }
        if( gcode->m == 83 ){ this->absolute_mode = false; }
        if( gcode->m == 84 ){ this->en_pin.set(1); }
        if (gcode->m == 92 ){
            if (gcode->has_letter('E')){
                this->steps_per_millimeter = gcode->get_value('E');
            }
        }
    }

    // The mode is OFF by default, and SOLO or FOLLOW only if we need to extrude
    this->mode = OFF;

    if( gcode->has_g ){
        // G92: Reset extruder position
        if( gcode->g == 92 ){
            gcode->mark_as_taken();
            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->unstepped_distance = 0;
            }else if( gcode->get_num_args() == 0){
                this->current_position = 0.0;
                this->target_position = this->current_position;
                this->unstepped_distance = 0;
            }
        }else if ((gcode->g == 0) || (gcode->g == 1)){
            // Extrusion length from 'G' Gcode
            if( gcode->has_letter('E' )){
                // Get relative extrusion distance depending on mode ( in absolute mode we must substract target_position )
                float extrusion_distance = gcode->get_value('E');
                float relative_extrusion_distance = extrusion_distance;
                if (this->absolute_mode)
                {
                    relative_extrusion_distance -= this->target_position;
                    this->target_position = extrusion_distance;
                }
                else
                {
                    this->target_position += relative_extrusion_distance;
                }

                // If the robot is moving, we follow it's movement, otherwise, we move alone
                if( fabs(gcode->millimeters_of_travel) < 0.0001 ){  // With floating numbers, we can have 0 != 0 ... beeeh. For more info see : http://upload.wikimedia.org/wikipedia/commons/0/0a/Cain_Henri_Vidal_Tuileries.jpg
                    this->mode = SOLO;
                    this->travel_distance = relative_extrusion_distance;
                }else{
                    // We move proportionally to the robot's movement
                    this->mode = FOLLOW;
                    this->travel_ratio = relative_extrusion_distance / gcode->millimeters_of_travel;
                    // TODO: check resulting flowrate, limit robot speed if it exceeds max_speed
                }

                this->en_pin.set(0);
            }
            if (gcode->has_letter('F'))
            {
                this->feed_rate = gcode->get_value('F');
                if (this->feed_rate > (this->max_speed * THEKERNEL->robot->seconds_per_minute))
                    this->feed_rate = this->max_speed * THEKERNEL->robot->seconds_per_minute;
                feed_rate /= THEKERNEL->robot->seconds_per_minute;
            }
        }else if( gcode->g == 90 ){ this->absolute_mode = true;
        }else if( gcode->g == 91 ){ this->absolute_mode = false;
        }
    }
}

// When a new block begins, either follow the robot, or step by ourselves ( or stay back and do nothing )
void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);


    if( this->mode == SOLO ){
        // In solo mode we take the block so we can move even if the stepper has nothing to do

        this->current_position += this->travel_distance ;

        int steps_to_step = abs(int(floor(this->steps_per_millimeter * (this->travel_distance +this->unstepped_distance) )));

        if ( this->travel_distance > 0 ){
            this->unstepped_distance += this->travel_distance -(steps_to_step/this->steps_per_millimeter); //catch any overflow
        }   else {
            this->unstepped_distance += this->travel_distance +(steps_to_step/this->steps_per_millimeter); //catch any overflow
        }

        if( steps_to_step != 0 ){

            // We take the block, we have to release it or everything gets stuck
            block->take();
            this->current_block = block;

            this->stepper_motor->steps_per_second = 0;
            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step);

        }else{
            this->current_block = NULL;
        }

    }else if( this->mode == FOLLOW ){
        // In non-solo mode, we just follow the stepper module
        this->travel_distance = block->millimeters * this->travel_ratio;

        this->current_position += this->travel_distance;

        int steps_to_step = abs(int(floor(this->steps_per_millimeter * (this->travel_distance + this->unstepped_distance) )));

        if ( this->travel_distance > 0 ){
            this->unstepped_distance += this->travel_distance -(steps_to_step/this->steps_per_millimeter); //catch any overflow
        }   else {
            this->unstepped_distance += this->travel_distance +(steps_to_step/this->steps_per_millimeter); //catch any overflow
        }

        if( steps_to_step != 0 ){
            block->take();
            this->current_block = block;

            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step );
            this->on_speed_change(0); // initialise speed in case we get called first
        }else{
            this->current_block = NULL;
        }

    }else if( this->mode == OFF ){
        // No movement means we must reset our speed
        this->current_block = NULL;
        //this->stepper_motor->set_speed(0);

    }

}

// When a block ends, pause the stepping interrupt
void Extruder::on_block_end(void* argument){
    this->current_block = NULL;
}

// Called periodically to change the speed to match acceleration or to match the speed of the robot
uint32_t Extruder::acceleration_tick(uint32_t dummy){

    // Avoid trying to work when we really shouldn't ( between blocks or re-entry )
    if( this->current_block == NULL ||  this->paused || this->mode != SOLO ){ return 0; }

    uint32_t current_rate = this->stepper_motor->steps_per_second;
    uint32_t target_rate = int(floor(this->feed_rate * this->steps_per_millimeter));

    if( current_rate < target_rate ){
        uint32_t rate_increase = int(floor((this->acceleration/THEKERNEL->stepper->acceleration_ticks_per_second)*this->steps_per_millimeter));
        current_rate = min( target_rate, current_rate + rate_increase );
    }
    if( current_rate > target_rate ){ current_rate = target_rate; }

    // steps per second
    this->stepper_motor->set_speed(max(current_rate, THEKERNEL->stepper->minimum_steps_per_minute/60));

    return 0;
}

// Speed has been updated for the robot's stepper, we must update accordingly
void Extruder::on_speed_change( void* argument ){

    // Avoid trying to work when we really shouldn't ( between blocks or re-entry )
    if( this->current_block == NULL ||  this->paused || this->mode != FOLLOW || this->stepper_motor->moving != true ){ return; }

    /*
    * nominal block duration = current block's steps / ( current block's nominal rate / 60 )
    * nominal extruder rate = extruder steps / nominal block duration
    * actual extruder rate = nominal extruder rate * ( ( stepper's steps per minute / 60 ) / ( current block's nominal rate / 60 ) )
    * or actual extruder rate = ( ( extruder steps * ( current block's nominal_rate / 60 ) ) / current block's steps ) * ( ( stepper's steps per minute / 60 ) / ( current block's nominal rate / 60 ) )
    * or simplified : extruder steps * ( stepper's steps per minute / 60 ) ) / current block's steps
    * or even : ( stepper steps per minute / 60 ) * ( extruder steps / current block's steps )
    */

    this->stepper_motor->set_speed( max( ( THEKERNEL->stepper->trapezoid_adjusted_rate /60.0) * ( (float)this->stepper_motor->steps_to_move / (float)this->current_block->steps_event_count ), THEKERNEL->stepper->minimum_steps_per_minute/60.0 ) );

}



// When the stepper has finished it's move
uint32_t Extruder::stepper_motor_finished_move(uint32_t dummy){

    //printf("extruder releasing\r\n");

    if (this->current_block){ // this should always be true, but sometimes it isn't. TODO: find out why
        Block* block = this->current_block;
        this->current_block = NULL;
        block->release();
    }
    return 0;

}

