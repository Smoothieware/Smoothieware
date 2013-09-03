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

// Get config
void Endstops::on_config_reload(void* argument){
    this->pins[0].from_string(         this->kernel->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[1].from_string(         this->kernel->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[2].from_string(         this->kernel->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[3].from_string(         this->kernel->config->value(alpha_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[4].from_string(         this->kernel->config->value(beta_max_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[5].from_string(         this->kernel->config->value(gamma_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();

    // we need to know steps per mm for M206, also use them for all settings
    this->steps_per_mm[0]           =  this->kernel->config->value(alpha_steps_per_mm_checksum         )->as_number();
    this->steps_per_mm[1]           =  this->kernel->config->value(beta_steps_per_mm_checksum          )->as_number();
    this->steps_per_mm[2]           =  this->kernel->config->value(gamma_steps_per_mm_checksum         )->as_number();

    this->fast_rates[0]             =  this->kernel->config->value(alpha_fast_homing_rate_checksum     )->by_default(4000 )->as_number();
    this->fast_rates[1]             =  this->kernel->config->value(beta_fast_homing_rate_checksum      )->by_default(4000 )->as_number();
    this->fast_rates[2]             =  this->kernel->config->value(gamma_fast_homing_rate_checksum     )->by_default(6400 )->as_number();
    this->slow_rates[0]             =  this->kernel->config->value(alpha_slow_homing_rate_checksum     )->by_default(2000 )->as_number();
    this->slow_rates[1]             =  this->kernel->config->value(beta_slow_homing_rate_checksum      )->by_default(2000 )->as_number();
    this->slow_rates[2]             =  this->kernel->config->value(gamma_slow_homing_rate_checksum     )->by_default(3200 )->as_number();
    this->retract_steps[0]          =  this->kernel->config->value(alpha_homing_retract_checksum       )->by_default(400  )->as_number();
    this->retract_steps[1]          =  this->kernel->config->value(beta_homing_retract_checksum        )->by_default(400  )->as_number();
    this->retract_steps[2]          =  this->kernel->config->value(gamma_homing_retract_checksum       )->by_default(1600 )->as_number();

    // newer mm based config values override the old ones, convert to steps/mm and steps, defaults to what was set in the older config settings above
    this->fast_rates[0]=    this->kernel->config->value(alpha_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[0]  / steps_per_mm[0])->as_number() * steps_per_mm[0];
    this->fast_rates[1]=    this->kernel->config->value(beta_fast_homing_rate_mm_checksum  )->by_default(this->fast_rates[1]  / steps_per_mm[1])->as_number() * steps_per_mm[1];
    this->fast_rates[2]=    this->kernel->config->value(gamma_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[2]  / steps_per_mm[2])->as_number() * steps_per_mm[2];
    this->slow_rates[0]=    this->kernel->config->value(alpha_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[0]  / steps_per_mm[0])->as_number() * steps_per_mm[0];
    this->slow_rates[1]=    this->kernel->config->value(beta_slow_homing_rate_mm_checksum  )->by_default(this->slow_rates[1]  / steps_per_mm[1])->as_number() * steps_per_mm[1];
    this->slow_rates[2]=    this->kernel->config->value(gamma_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[2]  / steps_per_mm[2])->as_number() * steps_per_mm[2];
    this->retract_steps[0]= this->kernel->config->value(alpha_homing_retract_mm_checksum   )->by_default(this->retract_steps[0]/steps_per_mm[0])->as_number() * steps_per_mm[0];
    this->retract_steps[1]= this->kernel->config->value(beta_homing_retract_mm_checksum    )->by_default(this->retract_steps[1]/steps_per_mm[1])->as_number() * steps_per_mm[1];
    this->retract_steps[2]= this->kernel->config->value(gamma_homing_retract_mm_checksum   )->by_default(this->retract_steps[2]/steps_per_mm[2])->as_number() * steps_per_mm[2];

    this->debounce_count  = this->kernel->config->value(endstop_debounce_count_checksum    )->by_default(100)->as_number();
    this->lift_steps = this->kernel->config->value(calibrate_lift_checksum )->by_default(10)->as_number() * steps_per_mm[0];
    this->calibrate_radius = this->kernel->config->value(calibrate_radius_checksum )->by_default(50)->as_number();
    this->arm_radius = this->kernel->config->value(arm_radius_checksum )->by_default(124.0)->as_number();

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

    this->is_corexy                 =  this->kernel->config->value(corexy_homing_checksum)->by_default(false)->as_bool();
    this->is_delta                  =  this->kernel->config->value(delta_homing_checksum)->by_default(false)->as_bool();

    // endstop trim used by deltas to do soft adjusting, in mm, convert to steps, and negate depending on homing direction
    // eg on a delta homing to max, a negative trim value will move the carriage down, and a positive will move it up
    int dirx= (this->home_direction[0] ? 1 : -1);
    int diry= (this->home_direction[1] ? 1 : -1);
    int dirz= (this->home_direction[2] ? 1 : -1);
    this->trim[0]= this->kernel->config->value(alpha_trim_checksum )->by_default(0  )->as_number() * steps_per_mm[0] * dirx;
    this->trim[1]= this->kernel->config->value(beta_trim_checksum  )->by_default(0  )->as_number() * steps_per_mm[1] * diry;
    this->trim[2]= this->kernel->config->value(gamma_trim_checksum )->by_default(0  )->as_number() * steps_per_mm[2] * dirz;
}

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

void Endstops::wait_for_moves(){
    // Wait for moves to be done
    for( char c = 'X'; c <= 'Z'; c++ ){
        while( this->steppers[c - 'X']->moving ){
            this->kernel->call_event(ON_IDLE);
        }
    }
}


uint32_t Endstops::wait_for_ztouch(){
    bool running = true;
    unsigned int debounce = 0;
    uint32_t saved_steps = 0;
    while(running){
        running = false;
        this->kernel->call_event(ON_IDLE);
        if( this->pins[2].get() ){  //probe contact  //TODO configure probe pin?
            if( debounce < debounce_count ) { //debounce and loop
                debounce ++;
                running = true;
            } else { //fully debounced, stop all motors
                saved_steps = this->steppers[0]->stepped; //save the number of actually moved steps from alpha tower before tool counters get reset
                for( int i = 0; i <= 2; i++ ){ //TODO support more than 3 axis
                    this->steppers[i]->move(0,0);
                }
                return (saved_steps);
            }
        }else{
            // The endstop was not hit yet
            running = true;
            debounce = 0;
        }
    }

    return(0);
}

// this homing works for cartesian and delta printers, not for HBots/CoreXY
void Endstops::do_homing(char axes_to_move) {
    // Start moving the axes to the origin
    this->status = MOVING_TO_ORIGIN_FAST;
    for( char c = 'X'; c <= 'Z'; c++ ){
        if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
            this->steppers[c - 'X']->set_speed(this->fast_rates[c - 'X']);
            this->steppers[c - 'X']->move(this->home_direction[c - 'X'],10000000);
        }
    }

    // Wait for all axes to have homed
    this->wait_for_homed(axes_to_move);

    // Move back a small distance
    this->status = MOVING_BACK;
    bool inverted_dir;
    for( char c = 'X'; c <= 'Z'; c++ ){
        if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){
            inverted_dir = !this->home_direction[c - 'X'];
            this->steppers[c - 'X']->set_speed(this->slow_rates[c - 'X']);
            this->steppers[c - 'X']->move(inverted_dir,this->retract_steps[c - 'X']);
        }
    }

     // Wait for moves to be done
    for( char c = 'X'; c <= 'Z'; c++ ){
        if(  ( axes_to_move >> ( c - 'X' ) ) & 1 ){
            while( this->steppers[c - 'X']->moving ){
                this->kernel->call_event(ON_IDLE);
            }
        }
    }

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

    if(this->is_delta) {
        // move for soft trim
        this->status = MOVING_BACK;
        for( char c = 'X'; c <= 'Z'; c++ ){
            if( this->trim[c - 'X'] != 0 && ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                inverted_dir = !this->home_direction[c - 'X'];
                // move up or down depending on sign of trim
                if(this->trim[c - 'X'] < 0) inverted_dir= !inverted_dir;
                this->steppers[c - 'X']->set_speed(this->slow_rates[c - 'X']);
                this->steppers[c - 'X']->move(inverted_dir,this->trim[c - 'X']);
            }
        }

        // Wait for moves to be done
        for( char c = 'X'; c <= 'Z'; c++ ){
            if(  ( axes_to_move >> ( c - 'X' ) ) & 1 ){
                //this->kernel->streams->printf("axis %c \r\n", c );
                while( this->steppers[c - 'X']->moving ){
                    this->kernel->call_event(ON_IDLE);
                }
            }
        }
    }

    // Homing is done
    this->status = NOT_HOMING;
}

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

void Endstops::wait_for_homed_corexy(int axis){
    bool running = true;
    unsigned int debounce[3] = {0,0,0};
    while(running){
        running = false;
        this->kernel->call_event(ON_IDLE);
        if( this->pins[axis + (this->home_direction[axis]?0:3)].get() ){
            if( debounce[axis] < debounce_count ) {
                debounce[axis] ++;
                running = true;
            } else {
                // turn both off if running
                if(this->steppers[X_AXIS]->moving) this->steppers[X_AXIS]->move(0,0);
                if(this->steppers[Y_AXIS]->moving) this->steppers[Y_AXIS]->move(0,0);
            }
        }else{
            // The endstop was not hit yet
            running = true;
            debounce[axis] = 0;
        }
    }
}

// this homing works for HBots/CoreXY
void Endstops::do_homing_corexy(char axes_to_move) {
    // Start moving the axes to the origin
    if(axes_to_move & 0x01) { // Home X, which means both X and Y in same direction
        this->status = MOVING_TO_ORIGIN_FAST;
        this->steppers[X_AXIS]->set_speed(this->fast_rates[X_AXIS]);
        this->steppers[X_AXIS]->move(this->home_direction[X_AXIS], 10000000);
        this->steppers[Y_AXIS]->set_speed(this->fast_rates[X_AXIS]);
        this->steppers[Y_AXIS]->move(this->home_direction[X_AXIS], 10000000);

        // wait for X
        this->wait_for_homed_corexy(X_AXIS);

        // Move back a small distance
        this->status = MOVING_BACK;
        this->steppers[X_AXIS]->set_speed(this->slow_rates[X_AXIS]);
        this->steppers[X_AXIS]->move(!this->home_direction[X_AXIS], this->retract_steps[X_AXIS]);
        this->steppers[Y_AXIS]->set_speed(this->slow_rates[X_AXIS]);
        this->steppers[Y_AXIS]->move(!this->home_direction[X_AXIS], this->retract_steps[X_AXIS]);

        // wait until done
        while( this->steppers[X_AXIS]->moving ){ this->kernel->call_event(ON_IDLE); }
        while( this->steppers[Y_AXIS]->moving ){ this->kernel->call_event(ON_IDLE); }

        // Start moving the axes to the origin slowly
        this->status = MOVING_TO_ORIGIN_SLOW;
        this->steppers[X_AXIS]->set_speed(this->slow_rates[X_AXIS]);
        this->steppers[X_AXIS]->move(this->home_direction[X_AXIS], 10000000);
        this->steppers[Y_AXIS]->set_speed(this->slow_rates[X_AXIS]);
        this->steppers[Y_AXIS]->move(this->home_direction[X_AXIS], 10000000);

        // wait for X
        this->wait_for_homed_corexy(X_AXIS);
    }

    if(axes_to_move & 0x02) { // Home Y, which means both X and Y in different directions
        this->status = MOVING_TO_ORIGIN_FAST;
        this->steppers[X_AXIS]->set_speed(this->fast_rates[Y_AXIS]);
        this->steppers[X_AXIS]->move(this->home_direction[Y_AXIS], 10000000);
        this->steppers[Y_AXIS]->set_speed(this->fast_rates[Y_AXIS]); // yes I use X_axis speed as they need to go at the same speed
        this->steppers[Y_AXIS]->move(!this->home_direction[Y_AXIS], 10000000);

        // wait for Y
        this->wait_for_homed_corexy(Y_AXIS);

        // Move back a small distance
        this->status = MOVING_BACK;
        this->steppers[X_AXIS]->set_speed(this->slow_rates[Y_AXIS]);
        this->steppers[X_AXIS]->move(!this->home_direction[Y_AXIS], this->retract_steps[Y_AXIS]);
        this->steppers[Y_AXIS]->set_speed(this->slow_rates[Y_AXIS]);
        this->steppers[Y_AXIS]->move(this->home_direction[Y_AXIS], this->retract_steps[Y_AXIS]);

        // wait until done
        while( this->steppers[X_AXIS]->moving ){ this->kernel->call_event(ON_IDLE); }
        while( this->steppers[Y_AXIS]->moving ){ this->kernel->call_event(ON_IDLE); }

        // Start moving the axes to the origin slowly
        this->status = MOVING_TO_ORIGIN_SLOW;
        this->steppers[X_AXIS]->set_speed(this->slow_rates[Y_AXIS]);
        this->steppers[X_AXIS]->move(this->home_direction[Y_AXIS], 10000000);
        this->steppers[Y_AXIS]->set_speed(this->slow_rates[Y_AXIS]);
        this->steppers[Y_AXIS]->move(!this->home_direction[Y_AXIS], 10000000);

        // wait for Y
        this->wait_for_homed_corexy(Y_AXIS);
    }

    if(axes_to_move & 0x04) { // move Z
        do_homing(0x04); // just home normally for Z
    }

    // Homing is done
    this->status = NOT_HOMING;
}

void Endstops::move_all(bool direction, bool speed, unsigned int steps){
    bool move_dir;
    for( char c = 'X'; c <= 'Z'; c++ ){
        move_dir = this->home_direction[c - 'X']^direction ;
        if(speed){
            this->steppers[c - 'X']->set_speed(this->fast_rates[c - 'X']);
        } else {
            this->steppers[c - 'X']->set_speed(this->slow_rates[c - 'X']);
        }
        this->steppers[c - 'X']->move(move_dir,steps);
    }
}

// auto calibration routine for delta bots
void Endstops::calibrate_delta( StreamOutput *stream){

    uint32_t current_z_steps = 0;

    uint32_t originHeight = 0;
    long tower1Height = 0;
    long tower2Height = 0;
    long tower3Height = 0;
    double targetX = 0;
    double targetY = 0;

    char buf[32];
    int buffered_length = 0;
    string g;

/*
  #define PROBE_RADIUS  80
  #define DELTA_RADIUS_CORRECTION_FACTOR 0.15
*/
    // First wait for the queue to be empty
    this->kernel->conveyor->wait_for_empty_queue();
    // Enable the motors
    this->kernel->stepper->turn_enable_pins_on();
// home axis

    buffered_length = snprintf(buf, sizeof(buf), "G21");
    g.assign(buf, buffered_length);
    send_gcode(g);

    buffered_length = snprintf(buf, sizeof(buf), "G90");
    g.assign(buf, buffered_length);
    send_gcode(g);

    // deploy probe, check or abort
    if( this->pins[2].get() ){  //TODO configure probe pin?
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }

    //probe min z height at 0,0
    this->move_all(DOWN,FAST,10000000);
    current_z_steps += this->wait_for_ztouch();

    this->move_all(UP,FAST,this->retract_steps[0]); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps -= this->retract_steps[0];

    this->move_all(DOWN,SLOW,10000000);
    current_z_steps += this->wait_for_ztouch();

    originHeight = current_z_steps;

    //lift and reposition
    this->move_all(UP,FAST,this->lift_steps); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps -= this->lift_steps;

    // update planner position so we can make coordinated moves
        this->kernel->robot->reset_axis_position( this->homing_position[2] - (current_z_steps / this->steps_per_mm[0]) , 2);

    targetY = this->calibrate_radius * cos(240 * (3.141592653589793/180));
    targetX = this->calibrate_radius * sin(240 * (3.141592653589793/180));
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX,targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    this->kernel->conveyor->wait_for_empty_queue();


    // check probe retraction
    if( this->pins[2].get() ){  //TODO configure probe pin?
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }

    //probe tower 1 
    this->move_all(DOWN,FAST,10000000);
    current_z_steps += this->wait_for_ztouch();

    this->move_all(UP,FAST,this->retract_steps[0]); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps -= this->retract_steps[0];

    this->move_all(DOWN,SLOW,10000000);
    current_z_steps += this->wait_for_ztouch();

    tower1Height = current_z_steps;    

    //lift and reposition
    this->move_all(UP,FAST, current_z_steps - (originHeight - this->lift_steps)); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps = originHeight - this->lift_steps;

    targetY = this->calibrate_radius * cos(120 * (3.141592653589793/180));
    targetX = this->calibrate_radius * sin(120 * (3.141592653589793/180));
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX,targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    this->kernel->conveyor->wait_for_empty_queue();


    // check probe retraction
    if( this->pins[2].get() ){  //TODO configure probe pin?
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }    

    //probe tower 2
    this->move_all(DOWN,FAST,10000000);
    current_z_steps += this->wait_for_ztouch();

    this->move_all(UP,FAST,this->retract_steps[0]); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps -= this->retract_steps[0];

    this->move_all(DOWN,SLOW,10000000);
    current_z_steps += this->wait_for_ztouch();

    tower2Height = current_z_steps;    

    //lift and reposition
    this->move_all(UP,FAST, current_z_steps - (originHeight-this->lift_steps)); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps = originHeight - this->lift_steps;

    targetY = this->calibrate_radius * cos(0 * (3.141592653589793/180));
    targetX = this->calibrate_radius * sin(0 * (3.141592653589793/180));
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX,targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    this->kernel->conveyor->wait_for_empty_queue();    

    // check probe retraction
    if( this->pins[2].get() ){  //TODO configure probe pin?
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }  

    //probe tower 3 
    this->move_all(DOWN,FAST,10000000);
    current_z_steps += this->wait_for_ztouch();

    this->move_all(UP,FAST,this->retract_steps[0]); //alpha tower retract distance for all
    this->wait_for_moves();
    current_z_steps -= this->retract_steps[0];

    this->move_all(DOWN,SLOW,10000000);
    current_z_steps += this->wait_for_ztouch();

    tower3Height = current_z_steps;        

//retract probe

    //calculate 3 tower trim levels
    
    float centerAverage = (tower1Height + tower2Height + tower3Height)/3;

    long lowestTower = min(tower1Height,min(tower2Height,tower3Height));
    stream->printf("Detected offsets:\r\n");

    stream->printf("Origin Offset: %f.3\r\n", (centerAverage - originHeight)/this->steps_per_mm[0]); 
    stream->printf("T1CV: %f.3\r\n", -((tower1Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius)); 
    stream->printf("T2CV: %f.3\r\n", -((tower2Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius)); 
    stream->printf("T3CV: %f.3\r\n", -((tower3Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius)); 

    float t1Trim = -((tower1Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius);
    float t2Trim = -((tower2Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius);
    float t3Trim = -((tower3Height-lowestTower) / this->steps_per_mm[0])* ( this->arm_radius*.6/ calibrate_radius);
    t1Trim += trim[0];
    t2Trim += trim[1];
    t3Trim += trim[2];
    float maxTrim = max(t1Trim,max(t2Trim,t3Trim));
    t1Trim -= maxTrim;
    t2Trim -= maxTrim;
    t3Trim -= maxTrim;
 
    stream->printf("Calibrated Values:\r\n");
    stream->printf("Origin Height:%f.3\r\n",  originHeight/this->steps_per_mm[0] );//+ printer_state.probe_offset); 
    stream->printf("Delta Radius:%f.3\r\n", ((centerAverage - originHeight)/this->steps_per_mm[0])/0.15 + this->arm_radius); 
    stream->printf("T1trim:%f.3\r\n", t1Trim); 
    stream->printf("T2trim:%f.3\r\n", t2Trim); 
    stream->printf("T3trim:%f.3\r\n", t3Trim);

// apply values
    /*
      printer_state.delta_radius = ((centerAverage - originHeight)/axis_steps_per_unit[0])/DELTA_RADIUS_CORRECTION_FACTOR + printer_state.delta_radius;
      printer_state.tower1_trim = t1Trim ; 
      printer_state.tower2_trim = t2Trim ; 
      printer_state.tower3_trim = t3Trim ; 
      printer_state.zLength = abs( originHeight/axis_steps_per_unit[0]) + printer_state.probe_offset;
    */

}

// Start homing sequences by response to GCode commands
void Endstops::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_g)
    {
        if( gcode->g == 28 )
        {
            gcode->mark_as_taken();
            // G28 is received, we have homing to do

            // First wait for the queue to be empty
            this->kernel->conveyor->wait_for_empty_queue();

            // Do we move select axes or all of them
            char axes_to_move = 0;
            // only enable homing if the endstop is defined, deltas always home all axis
            bool home_all= this->is_delta || !( gcode->has_letter('X') || gcode->has_letter('Y') || gcode->has_letter('Z') );

            for( char c = 'X'; c <= 'Z'; c++ ){
                if( (home_all || gcode->has_letter(c)) && this->pins[c - 'X' + (this->home_direction[c - 'X']?0:3)].connected() ){ axes_to_move += ( 1 << (c - 'X' ) ); }
            }

            // Enable the motors
            this->kernel->stepper->turn_enable_pins_on();

            // do the actual homing
            if(is_corexy)
                do_homing_corexy(axes_to_move);
            else
                do_homing(axes_to_move);

            // Zero the ax(i/e)s position
            for( char c = 'X'; c <= 'Z'; c++ ){
                if( ( axes_to_move >> ( c - 'X' ) ) & 1 ){

                    this->kernel->robot->reset_axis_position(this->homing_position[c - 'X'], c - 'X');
                }
            }

        } else if (gcode->g == 32 )
        {
            gcode->mark_as_taken();
            this->calibrate_delta(gcode->stream);
        }


    }
    else if (gcode->has_m){
        switch(gcode->m){
            case 119:
                {

                    int px= this->home_direction[0] ? 0 : 3;
                    int py= this->home_direction[1] ? 1 : 4;
                    int pz= this->home_direction[2] ? 2 : 5;
                    const char* mx= this->home_direction[0] ? "min" : "max";
                    const char* my= this->home_direction[1] ? "min" : "max";
                    const char* mz= this->home_direction[2] ? "min" : "max";

                    gcode->stream->printf("X %s:%d Y %s:%d Z %s:%d\n", mx, this->pins[px].get(), my, this->pins[py].get(), mz, this->pins[pz].get());
                    gcode->mark_as_taken();
                }
                break;

            case 206: // M206 - set trim for each axis in mm
                {
                    int dirx= (this->home_direction[0] ? 1 : -1);
                    int diry= (this->home_direction[1] ? 1 : -1);
                    int dirz= (this->home_direction[2] ? 1 : -1);
                    double mm[3];
                    mm[0]= trim[0]/steps_per_mm[0] * dirx; // convert to mm
                    mm[1]= trim[1]/steps_per_mm[1] * diry;
                    mm[2]= trim[2]/steps_per_mm[2] * dirz;

                    if(gcode->has_letter('X')) mm[0]= gcode->get_value('X');
                    if(gcode->has_letter('Y')) mm[1]= gcode->get_value('Y');
                    if(gcode->has_letter('Z')) mm[2]= gcode->get_value('Z');

                    trim[0]= lround(mm[0]*steps_per_mm[0]) * dirx; // convert back to steps
                    trim[1]= lround(mm[1]*steps_per_mm[1]) * diry;
                    trim[2]= lround(mm[2]*steps_per_mm[2]) * dirz;

                    // print the current trim values in mm and steps
                    char buf[64];
                    int n= snprintf(buf, sizeof(buf), "X:%5.3f (%d) Y:%5.3f (%d) Z:%5.3f (%d) ", mm[0], trim[0], mm[1], trim[1], mm[2], trim[2]);
                    gcode->txt_after_ok.append(buf, n);
                    gcode->mark_as_taken();
                }
                break;

        }
    }
}

void Endstops::send_gcode(std::string g) {
    Gcode gcode(g, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode );
}
