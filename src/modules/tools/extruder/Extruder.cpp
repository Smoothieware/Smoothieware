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

#define extruder_step_pin_checksum    40763
#define extruder_dir_pin_checksum     57277
#define extruder_en_pin_checksum      8017

Extruder::Extruder() {
    this->absolute_mode = true;
    this->direction     = 1;
    this->acceleration_lock = false;
    this->step_counter = 0;
    this->counter_increment = 0;
    this->paused = false;
}

void Extruder::on_module_loaded() {

    // Do not do anything if not enabledd
    if( this->kernel->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() == false ){ return; }

    // Settings
    this->on_config_reload(this);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);

    // Start values
    this->start_position = 0;
    this->target_position = 0;
    this->current_position = 0;
    this->current_block = NULL;
    this->mode = OFF;

    // Update speed every *acceleration_ticks_per_second*
    // TODO: Make this an independent setting
    this->kernel->slow_ticker->attach( this->kernel->stepper->acceleration_ticks_per_second , this, &Extruder::acceleration_tick );

    // Initiate main_interrupt timer and step reset timer
    this->kernel->step_ticker->attach( this, &Extruder::stepping_tick );
    this->kernel->step_ticker->reset_attach( this, &Extruder::reset_step_pin );

}

// Get config
void Extruder::on_config_reload(void* argument){
	this->microseconds_per_step_pulse = this->kernel->config->value(microseconds_per_step_pulse_checksum)->by_default(5)->as_number();
	this->steps_per_millimeter        = this->kernel->config->value(extruder_steps_per_mm_checksum       )->by_default(1)->as_number();
	this->feed_rate                   = this->kernel->config->value(default_feed_rate_checksum          )->by_default(1)->as_number();
	this->acceleration                = this->kernel->config->value(acceleration_checksum               )->by_default(1)->as_number();

	this->step_pin                    = this->kernel->config->value(extruder_step_pin_checksum          )->by_default("1.22" )->as_pin()->as_output();
	this->dir_pin                     = this->kernel->config->value(extruder_dir_pin_checksum           )->by_default("1.19" )->as_pin()->as_output();
	this->en_pin                      = this->kernel->config->value(extruder_en_pin_checksum            )->by_default("0.19" )->as_pin()->as_output();
}


// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Extruder::on_pause(void* argument){
    this->paused = true;
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Extruder::on_play(void* argument){
    this->paused = false;
}



// Compute extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    // Absolute/relative mode
    if( gcode->has_letter('M')){
        int code = (int) gcode->get_value('M');
        if( code == 82 ){ this->absolute_mode = true; }
        if( code == 83 ){ this->absolute_mode = false; }
        if( code == 84 ){ this->en_pin->set(0); }
    }

    // The mode is OFF by default, and SOLO or FOLLOW only if we need to extrude
    this->mode = OFF;

    if( gcode->has_letter('G') ){
        // G92: Reset extruder position
        if( gcode->get_value('G') == 92 ){
            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->start_position   = this->current_position;
            }
        }else{
            // Extrusion length from 'G' Gcode
            if( gcode->has_letter('E' )){
                // Get relative extrusion distance depending on mode ( in absolute mode we must substract target_position )
                double relative_extrusion_distance = gcode->get_value('E');
                if( this->absolute_mode == true ){ relative_extrusion_distance = relative_extrusion_distance - this->target_position; }

                // If the robot is moving, we follow it's movement, otherwise, we move alone
                if( fabs(gcode->millimeters_of_travel) < 0.0001 ){  // With floating numbers, we can have 0 != 0 ... beeeh
                    this->mode = SOLO;
                    this->travel_distance = relative_extrusion_distance;
                    if( gcode->has_letter('F') ){ this->feed_rate = gcode->get_value('F'); }
                }else{
                    this->mode = FOLLOW;
                    // We move proportionally to the robot's movement
                    this->travel_ratio = relative_extrusion_distance / gcode->millimeters_of_travel;
                }

                this->en_pin->set(1);
            }
        }
    }

}

// When a new block begins, either follow the robot, or step by ourselves ( or stay back and do nothing )
void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);
    if( this->mode == SOLO ){
        // In solo mode we take the block so we can move even if the stepper has nothing to do
        block->take();
        this->current_block = block;
        this->start_position = this->target_position;
        this->target_position = this->start_position + this->travel_distance ;
        this->travel_ratio = 0.2;   // TODO : Make a real acceleration thing
        if( this->target_position > this->current_position ){ this->direction = 1; }else if( this->target_position < this->current_position ){ this->direction = -1; }
        this->set_speed(int(floor((this->feed_rate/60)*this->steps_per_millimeter)));//Speed in steps per second
    }else if( this->mode == FOLLOW ){
        // In non-solo mode, we just follow the stepper module
        this->current_block = block;
        this->start_position = this->target_position;
        this->target_position =  this->start_position + ( this->current_block->millimeters * this->travel_ratio );
        if( this->target_position > this->current_position ){ this->direction = 1; }else if( this->target_position < this->current_position ){ this->direction = -1; }
        this->acceleration_tick(0);
    }

}

// When a block ends, pause the stepping interrupt
void Extruder::on_block_end(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = NULL;
}

// Called periodically to change the speed to match acceleration or to match the speed of the robot
uint32_t Extruder::acceleration_tick(uint32_t dummy){

    // Avoid trying to work when we really shouldn't ( between blocks or re-entry )
    if( this->current_block == NULL || this->acceleration_lock || this->paused ){ return 0; }
    this->acceleration_lock = true;

    // In solo mode, we mode independently from the robot
    if( this->mode == SOLO ){
        // TODO : Do real acceleration here
        this->travel_ratio += 0.03;
        if( this->travel_ratio > 1 ){ this->travel_ratio = 1; }
        this->set_speed( int(floor(((this->feed_rate/60)*this->steps_per_millimeter)*this->travel_ratio)) );  // Speed in steps per second

        // In follow mode we match the speed of the robot, + eventually advance
    }else if( this->mode == FOLLOW ){
        Stepper* stepper = this->kernel->stepper; // Just for convenience

        // Strategy :
        //   * Find where in the block will the stepper be at the next tick ( if the block will have ended then, don't change speed )
        //   * Find what position this is for us
        //   * Find what speed we must go at to be at that position for the next acceleration tick
        // TODO : This works, but PLEASE PLEASE PLEASE if you know a better way to do it, do it better, I don't find this elegant at all, it's just the best I could think of
        // UPDATE: Yes, this sucks, I have ideas on how to do it better. If this is really bugging you, open a ticket and I'll make it a priority

        int ticks_forward = 3;
        // We need to take those values here, and then use those instead of the live values, because using the live values inside the loop can break things ( infinite loops etc ... )
        double next_stepper_rate = stepper->trapezoid_adjusted_rate;
        double step_events_completed =   (double(double(stepper->step_events_completed)/double(1<<16)));
        double position = ( this->current_position - this->start_position ) * this->direction ;
        double length = fabs( this->start_position - this->target_position );
        double last_ratio = -1;

        // Do the startegy above, but if it does not work, look a bit further and try again, and again ...
        while(1){

            // Find the position where we should be at the next tick
            double next_ratio = double( step_events_completed + ( next_stepper_rate / 60 / ((double(stepper->acceleration_ticks_per_second)/ticks_forward)) ) ) / double( this->current_block->steps_event_count );
            double next_relative_position = ( length * next_ratio );

            // Advance
            // TODO: Proper advance configuration
            double advance = double(next_stepper_rate) * ( 0.00001 * 0.15 ) * 0.4 ;
            //double advance = 0;
            next_relative_position += ( advance );

            // TODO : all of those "if->return" is very hacky, we should do the math in a way where most of those don't happen, but that requires doing tons of drawing ...
            if( last_ratio == next_ratio ){ this->acceleration_lock = false; return 0; }else{ last_ratio = next_ratio; }
            if( next_ratio == 0 || next_ratio > 1 ){ this->acceleration_lock = false; return 0; }
            if( ticks_forward > 1000 ){ this->acceleration_lock = false; return 0; } // This is very ugly

            // Hack : We have not looked far enough, we compute how far ahead we must look to get a relevant value
            if( position > next_relative_position ){
                double far_back = position - next_relative_position;
                double far_back_ratio = far_back / length;
                double move_duration = double( this->current_block->steps_event_count ) / ( double(next_stepper_rate) / 60 ) ;
                double ticks_in_a_move = round( stepper->acceleration_ticks_per_second * move_duration );
                double ratio_per_tick = 1 / ticks_in_a_move;
                double ticks_to_equilibrium = ceil(far_back_ratio / ratio_per_tick) + 1;
                ticks_forward += ticks_to_equilibrium;
                // Because this is a loop, and we can be interrupted by the stepping interrupt, if that interrupt changes block, the new block may not be solo, and we may get trapped into an infinite loop
                if( this->mode != FOLLOW ){ this->acceleration_lock = false; return 0; }
                continue;
            }

            // Finally, compute the speed to get to that next position
            double next_absolute_position = this->start_position + ( this->direction * next_relative_position );
            double steps_to_next_tick = ( next_relative_position - position ) * this->steps_per_millimeter;
            double speed_to_next_tick = steps_to_next_tick / ( 1 / double(double(this->kernel->stepper->acceleration_ticks_per_second) / ticks_forward) );

            // Change stepping speed
            this->set_speed( speed_to_next_tick );

            this->acceleration_lock = false;
            return 0;
        }
    }

    this->acceleration_lock = false;
	return 0;
}

// Convenience function to set stepping speed
void Extruder::set_speed( int steps_per_second ){

    if( steps_per_second < 10 ){ steps_per_second = 10; }

    // TODO : Proper limit config value
    if( steps_per_second > (this->feed_rate*double(this->steps_per_millimeter))/60 ){
        steps_per_second = (this->feed_rate*double(this->steps_per_millimeter))/60;
    }

    this->counter_increment = int(floor(double(1<<16)/double(this->kernel->stepper->base_stepping_frequency / steps_per_second)));

}

inline uint32_t Extruder::stepping_tick(uint32_t dummy){
    if( this->paused ){ return 0; }

    this->step_counter += this->counter_increment;
    if( this->step_counter > 1<<16 ){
        this->step_counter -= 1<<16;

        // If we still have steps to do
        // TODO: Step using the same timer as the robot, and count steps instead of absolute float position
        if( ( this->current_position < this->target_position && this->direction == 1 ) || ( this->current_position > this->target_position && this->direction == -1 ) ){
            this->current_position += (double(double(1)/double(this->steps_per_millimeter)))*double(this->direction);
            this->dir_pin->set((this->direction > 0) ? 1 : 0);
            this->step_pin->set(1);
        }else{
            // Move finished
            if( this->mode == SOLO && this->current_block != NULL ){
                // In follow mode, the robot takes and releases the block, in solo mode we do
                this->current_block->release();
            }
        }
    }
    return 0;
}

uint32_t Extruder::reset_step_pin(uint32_t dummy){
    this->step_pin->set(0);
	return 0;
}
