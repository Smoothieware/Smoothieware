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

Endstops::Endstops(){
    this->status = NOT_HOMING;
}

void Endstops::on_module_loaded() {
    // Do not do anything if not enabled
    if( this->kernel->config->value( endstops_module_enable_checksum )->by_default(true)->as_bool() == false ){ return; }

    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);

    // Take StepperMotor objects from Robot and keep them here
    this->steppers[0] = this->kernel->robot->alpha_stepper_motor;
    this->steppers[1] = this->kernel->robot->beta_stepper_motor;
    this->steppers[2] = this->kernel->robot->gamma_stepper_motor;

    // Settings
    this->on_config_reload(this);

}

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// Get config
void Endstops::on_config_reload(void* argument){
    this->pins[0].from_string(         this->kernel->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->pins[1].from_string(         this->kernel->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->pins[2].from_string(         this->kernel->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->pins[3].from_string(         this->kernel->config->value(alpha_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->pins[4].from_string(         this->kernel->config->value(beta_max_endstop_checksum           )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->pins[5].from_string(         this->kernel->config->value(gamma_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input()->pull_up();
    this->fast_rates[0]             =  this->kernel->config->value(alpha_fast_homing_rate_checksum     )->by_default(500  )->as_number();
    this->fast_rates[1]             =  this->kernel->config->value(beta_fast_homing_rate_checksum      )->by_default(500  )->as_number();
    this->fast_rates[2]             =  this->kernel->config->value(gamma_fast_homing_rate_checksum     )->by_default(5    )->as_number();
    this->slow_rates[0]             =  this->kernel->config->value(alpha_slow_homing_rate_checksum     )->by_default(100  )->as_number();
    this->slow_rates[1]             =  this->kernel->config->value(beta_slow_homing_rate_checksum      )->by_default(100  )->as_number();
    this->slow_rates[2]             =  this->kernel->config->value(gamma_slow_homing_rate_checksum     )->by_default(5    )->as_number();
    this->retract_steps[0]          =  this->kernel->config->value(alpha_homing_retract_checksum       )->by_default(30   )->as_number();
    this->retract_steps[1]          =  this->kernel->config->value(beta_homing_retract_checksum        )->by_default(30   )->as_number();
    this->retract_steps[2]          =  this->kernel->config->value(gamma_homing_retract_checksum       )->by_default(10   )->as_number();
    this->debounce_count            =  this->kernel->config->value(endstop_debounce_count_checksum     )->by_default(100  )->as_number();
    
    // get homing direction and convert to boolean where true is home to min, and false is home to max
    int home_dir                    = get_checksum(this->kernel->config->value(alpha_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[0]         = home_dir != home_to_max_checksum;
    
    home_dir                        = get_checksum(this->kernel->config->value(beta_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[1]         = home_dir != home_to_max_checksum;
    
    home_dir                        = get_checksum(this->kernel->config->value(gamma_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[2]         = home_dir != home_to_max_checksum;
    
    this->homing_position[0]        =  this->home_direction[0]?this->kernel->config->value(alpha_min_checksum)->by_default(0)->as_number():this->kernel->config->value(alpha_max_checksum)->by_default(200)->as_number();
    this->homing_position[1]        =  this->home_direction[1]?this->kernel->config->value(beta_min_checksum )->by_default(0)->as_number():this->kernel->config->value(beta_max_checksum )->by_default(200)->as_number();;
    this->homing_position[2]        =  this->home_direction[2]?this->kernel->config->value(gamma_min_checksum)->by_default(0)->as_number():this->kernel->config->value(gamma_max_checksum)->by_default(200)->as_number();;
}

//#pragma GCC pop_options

void Endstops::wait_for_homed(char axes_to_move){
    bool running = true;
    unsigned int debounce[3] = {0,0,0};
    while(running){
        running = false;
        this->kernel->call_event(ON_IDLE);
        for( char c = 'X'; c <= 'Z'; c++ ){
            if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                if( this->pins[c - 'X' + (this->home_direction[c - 'X']?0:3)].get() ){
                    if( debounce[c - 'X'] < debounce_count ) {
                        debounce[c - 'X'] ++;
                        running = true;
                    } else if ( this->steppers[c - 'X']->moving ){
                        this->steppers[c - 'X']->move(0,0);
                        //this->kernel->streams->printf("move done %c\r\n", c);
                    }
                }else{
                    // The endstop was not hit yet
                    running = true;
                    debounce[c - 'X'] = 0;
                }
            }
        }
    }
}

// Start homing sequences by response to GCode commands
void Endstops::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_g)
    {
        if( gcode->g == 28 )
        {
            // G28 is received, we have homing to do

            // First wait for the queue to be empty
            this->kernel->conveyor->wait_for_empty_queue();

            // Do we move select axes or all of them
            char axes_to_move = 0;
            // only enable homing if the endstop is defined
            bool home_all= !( gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') );
            
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( (home_all || gcode->has_letter(c)) && this->pins[c - 'X' + (this->direction[c - 'X']?0:3)].connected() ){ axes_to_move += ( 1 << (c - 'X' ) ); }
            }

            // Enable the motors
            this->kernel->stepper->turn_enable_pins_on();

            // Start moving the axes to the origin
            this->status = MOVING_TO_ORIGIN_FAST;
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                    gcode->stream->printf("homing axis %c\r\n", c);
                    this->steppers[c - 'X']->set_speed(this->fast_rates[c - 'X']);
                    this->steppers[c - 'X']->move(this->home_direction[c - 'X'],10000000);
                }
            }

            // Wait for all axes to have homed
            this->wait_for_homed(axes_to_move);

            gcode->stream->printf("test a\r\n");
            // Move back a small distance
            this->status = MOVING_BACK;
            int inverted_dir;
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                    inverted_dir = -(this->home_direction[c - 'X'] - 1);
                    this->steppers[c - 'X']->set_speed(this->slow_rates[c - 'X']);
                    this->steppers[c - 'X']->move(inverted_dir,this->retract_steps[c - 'X']);
                }
            }

            gcode->stream->printf("test b\r\n");
            // Wait for moves to be done
            for( char c = 'X'; c <= 'Z'; c++ ){
                if(  ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                    this->kernel->streams->printf("axis %c \r\n", c );
                    while( this->steppers[c - 'X']->moving ){
                        this->kernel->call_event(ON_IDLE);
                    }
                }
            }

            gcode->stream->printf("test c\r\n");

            // Start moving the axes to the origin slowly
            this->status = MOVING_TO_ORIGIN_SLOW;
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                    this->steppers[c - 'X']->set_speed(this->slow_rates[c -'X']);
                    this->steppers[c - 'X']->move(this->home_direction[c - 'X'],10000000);
                }
            }

            // Wait for all axes to have homed
            this->wait_for_homed(axes_to_move);

            // Homing is done
            this->status = NOT_HOMING;

            // Zero the ax(i/e)s position
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){

                    this->kernel->robot->reset_axis_position(this->homing_position[c - 'X'], c - 'X');
                }
            }

        }
    }
    else if (gcode->has_m){
        switch(gcode->m){
            case 119:
                gcode->stream->printf("X min:%d max:%d Y min:%d max:%d Z min:%d max:%d\n", this->pins[0].get(), this->pins[3].get(), this->pins[1].get(), this->pins[4].get(), this->pins[2].get(), this->pins[5].get() );
                break;
        }
    }
}

