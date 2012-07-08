/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Player.h"
#include "Endstops.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib

Endstops::Endstops(){
    this->status = NOT_HOMING;
}

void Endstops::on_module_loaded() {
    this->register_for_event(ON_GCODE_RECEIVED);

    // Take StepperMotor objects from Robot and keep them here
    this->steppers[0] = this->kernel->robot->alpha_stepper_motor;
    this->steppers[1] = this->kernel->robot->beta_stepper_motor;
    this->steppers[2] = this->kernel->robot->gamma_stepper_motor;

    // Settings
    this->on_config_reload(this);
    
}

// Get config
void Endstops::on_config_reload(void* argument){
    this->pins[0]                    = this->kernel->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_pin()->as_input();
    this->pins[1]                    = this->kernel->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_pin()->as_input();
    this->pins[2]                    = this->kernel->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_pin()->as_input();
}

// Start homing sequences by response to GCode commands
void Endstops::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_letter('G' )){
        if( gcode->get_value('G') == 28 ){
            // G28 is received, we have homing to do  

            // First wait for the queue to be empty
            while(this->kernel->player->queue.size() > 0) { wait_us(500); }

            // Do we move select axes or all of them
            bool home_all_axes = true;
            if( gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') ){ home_all_axes = false; }
           
            // Start moving the axes to the origin
            this->status = MOVING_TO_ORIGIN_FAST; 
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( home_all_axes || gcode->has_letter(c) ) && this->pins[c - 'X']->connected() ){
                    this->steppers[c - 'X']->move(0,10000000); 
                    this->steppers[c - 'X']->set_speed(10000); 
                }
            }

            // Wait for all axes to have homed
            bool running = true;
            while(running){
                running = false; 
                for( char c = 'X'; c <= 'Z'; c++ ){
                    if( ( home_all_axes || gcode->has_letter(c) ) && this->pins[c - 'X']->connected() ){
                         if( this->pins[c - 'X']->get() ){
                            // The endstop was hit, stop moving
                            if( this->steppers[c - 'X']->moving ){ 
                                this->steppers[c - 'X']->move(0,0); 
                            }  
                        }else{
                            // The endstop was not hit yet
                            running = true;
                         }
                    }
                }
            }



            // Homing is done
            this->status = NOT_HOMING;
        
        }
    }
}

