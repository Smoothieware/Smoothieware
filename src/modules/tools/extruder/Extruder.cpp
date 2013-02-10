/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Player.h"
#include "modules/robot/Block.h"
#include "modules/tools/extruder/Extruder.h"

/* The extruder module controls a filament extruder for 3D printing: http://en.wikipedia.org/wiki/Fused_deposition_modeling
* It can work in two modes : either the head does not move, and the extruder moves the filament at a specified speed ( SOLO mode here )
* or the head moves, and the extruder moves plastic at a speed proportional to the movement of the head ( FOLLOW mode here ).
*/

Extruder::Extruder() {
    this->absolute_mode = true;
    this->step_counter = 0;
    this->counter_increment = 0;
    this->paused = false;
}

void Extruder::on_module_loaded() {

    // Do not do anything if not enabledd
    if( this->kernel->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() == false ){ return; }

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
    this->current_steps = 0;
    this->current_block = NULL;
    this->mode = OFF;

    // Update speed every *acceleration_ticks_per_second*
    // TODO: Make this an independent setting
    this->kernel->slow_ticker->attach( this->kernel->stepper->acceleration_ticks_per_second , this, &Extruder::acceleration_tick );

    // Stepper motor object for the extruder
    this->stepper_motor  = this->kernel->step_ticker->add_stepper_motor( new StepperMotor(&step_pin, &dir_pin, &en_pin) );
    this->stepper_motor->attach(this, &Extruder::stepper_motor_finished_move );

}

// Get config
void Extruder::on_config_reload(void* argument){
    this->microseconds_per_step_pulse = this->kernel->config->value(microseconds_per_step_pulse_checksum)->by_default(5)->as_number();
    this->steps_per_millimeter        = this->kernel->config->value(extruder_steps_per_mm_checksum      )->by_default(1)->as_number();
    this->feed_rate                   = this->kernel->config->value(default_feed_rate_checksum          )->by_default(1000)->as_number();
    this->acceleration                = this->kernel->config->value(extruder_acceleration_checksum      )->by_default(1000)->as_number();
    this->max_speed                   = this->kernel->config->value(extruder_max_speed_checksum         )->by_default(1000)->as_number();

    this->step_pin.from_string(         this->kernel->config->value(extruder_step_pin_checksum          )->by_default("nc" )->as_string())->as_output();
    this->dir_pin.from_string(          this->kernel->config->value(extruder_dir_pin_checksum           )->by_default("nc" )->as_string())->as_output();
    this->en_pin.from_string(           this->kernel->config->value(extruder_en_pin_checksum            )->by_default("nc" )->as_string())->as_output()->as_open_drain();

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

void Extruder::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m)
    {
        if (gcode->m == 114)
        {
            gcode->stream->printf("E:%4.1f ", this->current_position);
            gcode->add_nl = true;
        }
        if (gcode->m == 92 )
        {
            double spm = this->steps_per_millimeter;
            if (gcode->has_letter('E'))
                spm = gcode->get_value('E');
            gcode->stream->printf("E:%g ", spm);
            gcode->add_nl = true;
        }
    }
}

// Compute extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    // Absolute/relative mode
    if( gcode->has_m ){
        if( gcode->m == 82 ){ this->absolute_mode = true; }
        if( gcode->m == 83 ){ this->absolute_mode = false; }
        if( gcode->m == 84 ){ this->en_pin.set(1); }
        if (gcode->m == 92 )
        {
            if (gcode->has_letter('E'))
            {
                this->steps_per_millimeter = gcode->get_value('E');
                this->current_steps = int(floor(this->steps_per_millimeter * this->current_position));
            }
        }
    }

    // The mode is OFF by default, and SOLO or FOLLOW only if we need to extrude
    this->mode = OFF;

    if( gcode->has_g ){
        // G92: Reset extruder position
        if( gcode->g == 92 ){
            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->current_steps = int(floor(this->steps_per_millimeter * this->current_position));
            }else if( gcode->get_num_args() == 0){
                this->current_position = 0.0;
                this->target_position = this->current_position;
                this->current_steps = 0;
            }
        }else if ((gcode->g == 0) || (gcode->g == 1)){
            // Extrusion length from 'G' Gcode
            if( gcode->has_letter('E' )){
                // Get relative extrusion distance depending on mode ( in absolute mode we must substract target_position )
                double relative_extrusion_distance = gcode->get_value('E');
                if( this->absolute_mode == true ){ relative_extrusion_distance = relative_extrusion_distance - this->target_position; }

                // If the robot is moving, we follow it's movement, otherwise, we move alone
                if( fabs(gcode->millimeters_of_travel) < 0.0001 ){  // With floating numbers, we can have 0 != 0 ... beeeh. For more info see : http://upload.wikimedia.org/wikipedia/commons/0/0a/Cain_Henri_Vidal_Tuileries.jpg
                    this->mode = SOLO;
                    this->travel_distance = relative_extrusion_distance;
                    if( gcode->has_letter('F') ){ this->feed_rate = gcode->get_value('F'); }
                    if (this->feed_rate > (this->max_speed * 60))
                        this->feed_rate = this->max_speed * 60;
                }else{
                    // We move proportionally to the robot's movement
                    this->mode = FOLLOW;
                    this->travel_ratio = relative_extrusion_distance / gcode->millimeters_of_travel;
                    // TODO: check resulting flowrate, limit robot speed if it exceeds max_speed
                }

                this->en_pin.set(0);
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

        this->target_position = this->current_position + this->travel_distance ;

        //int32_t steps_to_step = abs( int( floor(this->steps_per_millimeter*this->target_position) - floor(this->steps_per_millimeter*this->current_position) ) );

        int old_steps = this->current_steps;
        int target_steps = int( floor(this->steps_per_millimeter*this->target_position) );
        int steps_to_step = abs( target_steps - old_steps );
        this->current_steps = target_steps;

        if( steps_to_step != 0 ){

            // We take the block, we have to release it or everything gets stuck
            block->take();
            this->current_block = block;

            this->stepper_motor->steps_per_second = 0;
            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step);

        }


    }else if( this->mode == FOLLOW ){
        // In non-solo mode, we just follow the stepper module

        this->current_block = block;
        this->target_position =  this->current_position + ( this->current_block->millimeters * this->travel_ratio );

        //int32_t steps_to_step = abs( int( floor(this->steps_per_millimeter*this->target_position) - floor(this->steps_per_millimeter*this->current_position) ) );

        int old_steps = this->current_steps;
        int target_steps = int( floor(this->steps_per_millimeter*this->target_position) );
        int steps_to_step = target_steps - old_steps ;
        this->current_steps = target_steps;


        if( steps_to_step != 0 ){

            //printf("taken for extruder: %u \r\n", steps_to_step);

            block->take();

            //printf("spm:%f td:%f steps:%d ( %f - %f ) \r\n", this->steps_per_millimeter, this->travel_distance,  steps_to_step, this->target_position, this->current_position  );

            this->stepper_motor->move( ( steps_to_step > 0 ), abs(steps_to_step) );



        }

    }else if( this->mode == OFF ){
        // No movement means we must reset our speed

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
    uint32_t target_rate = int(floor((this->feed_rate/60)*this->steps_per_millimeter));

    if( current_rate < target_rate ){
        uint32_t rate_increase = int(floor((this->acceleration/this->kernel->stepper->acceleration_ticks_per_second)*this->steps_per_millimeter));
        current_rate = min( target_rate, current_rate + rate_increase );
    }
    if( current_rate > target_rate ){ current_rate = target_rate; }

    this->stepper_motor->set_speed(max(current_rate, this->kernel->stepper->minimum_steps_per_minute/60));

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

    this->stepper_motor->set_speed( max( ( this->kernel->stepper->trapezoid_adjusted_rate /60L) * ( (double)this->stepper_motor->steps_to_move / (double)this->current_block->steps_event_count ), this->kernel->stepper->minimum_steps_per_minute/60 ) );

}



// When the stepper has finished it's move
uint32_t Extruder::stepper_motor_finished_move(uint32_t dummy){

    //printf("extruder releasing\r\n");

    this->current_position = this->target_position;

    if (this->current_block) // this should always be true, but sometimes it isn't. TODO: find out why
        this->current_block->release();
    return 0;

}

