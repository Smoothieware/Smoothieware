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
    this->en_pin.from_string(           this->kernel->config->value(extruder_en_pin_checksum            )->by_default("nc" )->as_string())->as_output();

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
        switch (gcode->m)
        {
            case 114: // M114 - report position
            {
                gcode->stream->printf("E:%4.1f ", this->current_position);
                gcode->add_nl = true;
                gcode->mark_as_taken();
                break;
            };
            case 82: // M82 - absolute extruder mode
            case 90: // M90 - absolute mode (all axes)
                this->absolute_mode = true;
                gcode->mark_as_taken();
                break;
            case 83: // M83 - relative extruder mode
            case 91: // M91 - relative mode (all axes)
                this->absolute_mode = false;
                gcode->mark_as_taken();
                break;

            // M84 must be executed in sequence, so we create a new Action
            case 84:{
                ExtruderData* data = allocate_action_data(ExtruderData, this);
                data->disable = true;
                data->solo_steps = 0;
                data->travel_ratio = 0.0;
                kernel->conveyor->next_action()->add_data(data);
                kernel->conveyor->commit_action();
                gcode->mark_as_taken();
                break;
            };

            case 92: // M92 - set steps per mm
            {
                if (gcode->has_letter('E')){
                    this->steps_per_millimeter = gcode->get_value('E');
                    this->current_steps = int(floor(this->steps_per_millimeter * this->current_position));
                }
                gcode->stream->printf("E:%g ", steps_per_millimeter);
                gcode->add_nl = true;
                gcode->mark_as_taken();
                break;
            };
        }
    }

    // Add to the queue for on_gcode_execute to process
    if( gcode->has_g){
        if( gcode->g == 92 ){
            gcode->mark_as_taken();
            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->current_steps    = int(floor(this->steps_per_millimeter * this->current_position));
            }else if( gcode->get_num_args() == 0){
                this->current_position = 0.0;
                this->target_position  = this->current_position;
                this->current_steps    = 0;
            }
        }
        if (gcode->g < 4)
        {
            if (gcode->has_letter('F'))
            {
                this->feed_rate = gcode->get_value('F');
                if (this->feed_rate > (this->max_speed * kernel->robot->seconds_per_minute))
                    this->feed_rate = this->max_speed * kernel->robot->seconds_per_minute;
                feed_rate /= kernel->robot->seconds_per_minute;
            }
            if (gcode->has_letter('E'))
            {
                double distance = gcode->get_value('E');

                double relative_distance = distance;
                if (absolute_mode)
                {
                    relative_distance -= target_position;
                    target_position = distance;
                }
                else
                {
                    target_position += relative_distance;
                }

                // we could use relative_distance but this way keeps more precision
                int steps = int(floor(distance * steps_per_millimeter));
                if (absolute_mode)
                    steps -= current_steps;

                ExtruderData* data = allocate_action_data(ExtruderData, this);
                data->travel_ratio = 0.0;
                data->solo_steps = 0;

                if ( fabs(gcode->millimeters_of_travel) < 0.0001 )
                {
                    // SOLO move
                    data->solo_steps = steps;
                    data->step_rate = this->feed_rate * this->steps_per_millimeter;
                    current_steps += steps;

                    // now we commit the action so we can do our move solo
                    kernel->conveyor->next_action()->add_data(data);
                    kernel->conveyor->commit_action();
                }
                else
                {
                    // FOLLOW move
                    data->travel_ratio = relative_distance / gcode->millimeters_of_travel;
                    kernel->conveyor->next_action()->add_data(data);
                }
            }
        }
    }else{
        // This is for follow move

    }
}

void Extruder::on_action_invoke(ActionData* argument)
{
    ExtruderData* data = static_cast<ExtruderData*>(argument);

    // TODO: something intelligent. maybe fill class variables from our data?
    if (data->solo_steps)
    {
        this->mode = SOLO;

        this->stepper_motor->set_speed(data->step_rate);
        this->stepper_motor->move( data->solo_steps > 0, labs(data->solo_steps));
    }
    else
    {
        this->travel_ratio = data->travel_ratio;
        this->mode = FOLLOW;
    }
}

// // Append an empty block in the queue so that solo mode can pick it up
// Block* Extruder::append_empty_block(){
//     this->kernel->conveyor->wait_for_queue(2);
//     Block* block = this->kernel->conveyor->new_block();
//     block->planner = this->kernel->planner;
//     block->millimeters = 0;
//     block->steps[0] = 0;
//     block->steps[1] = 0;
//     block->steps[2] = 0;
//     // feed the block into the system. Will execute it if we are at the beginning of the queue
//     block->ready();
//
//     return block;
// }

// When a new block begins, either follow the robot, or step by ourselves ( or stay back and do nothing )
void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);


    if( this->mode == FOLLOW ){
        // In non-solo mode, we just follow the stepper module
        this->travel_distance = block->millimeters * this->travel_ratio;

        this->current_position += this->travel_distance;

        int steps_to_step = abs(int(floor(this->steps_per_millimeter * this->travel_distance)));

        if( steps_to_step != 0 ){
            block->take();
            this->current_block = block;

            this->stepper_motor->move( ( this->travel_distance > 0 ), steps_to_step );
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
        uint32_t rate_increase = int(floor((this->acceleration/this->kernel->stepper->acceleration_ticks_per_second)*this->steps_per_millimeter));
        current_rate = min( target_rate, current_rate + rate_increase );
    }
    if( current_rate > target_rate ){ current_rate = target_rate; }

    // steps per second
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

    if (this->current_block)
    {
        Block* block = this->current_block;
        this->current_block = NULL;
        block->release();
    }
    if (this->mode == SOLO)
    {
        data->finish();
        data = NULL;
    }
    return 0;

}

