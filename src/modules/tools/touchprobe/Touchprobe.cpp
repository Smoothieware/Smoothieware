/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Touchprobe.h"

#include "BaseSolution.h"

#define touchprobe_enable_checksum           CHECKSUM("touchprobe_enable")
#define touchprobe_log_enable_checksum       CHECKSUM("touchprobe_log_enable")
#define touchprobe_logfile_name_checksum     CHECKSUM("touchprobe_logfile_name")
#define touchprobe_log_rotate_mcode_checksum CHECKSUM("touchprobe_log_rotate_mcode")
#define touchprobe_pin_checksum              CHECKSUM("touchprobe_pin")
#define touchprobe_debounce_count_checksum   CHECKSUM("touchprobe_debounce_count")

#define touchprobe_enable_checksum           CHECKSUM("touchprobe_enable")
#define touchprobe_log_enable_checksum       CHECKSUM("touchprobe_log_enable")
#define touchprobe_logfile_name_checksum     CHECKSUM("touchprobe_logfile_name")
#define touchprobe_log_rotate_mcode_checksum CHECKSUM("touchprobe_log_rotate_mcode")
#define touchprobe_pin_checksum              CHECKSUM("touchprobe_pin")
#define touchprobe_debounce_count_checksum   CHECKSUM("touchprobe_debounce_count")


void Touchprobe::on_module_loaded() {
    // if the module is disabled -> do nothing
    this->enabled = THEKERNEL->config->value( touchprobe_enable_checksum )->by_default(false)->as_bool();
    if( !(this->enabled) ){
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    this->probe_rate = 5;
    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_CONFIG_RELOAD);
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_IDLE);
}

void Touchprobe::on_config_reload(void* argument){
    this->pin.from_string(  THEKERNEL->config->value(touchprobe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->debounce_count =  THEKERNEL->config->value(touchprobe_debounce_count_checksum)->by_default(100  )->as_number();

    this->steppers[0] = THEKERNEL->robot->alpha_stepper_motor;
    this->steppers[1] = THEKERNEL->robot->beta_stepper_motor;
    this->steppers[2] = THEKERNEL->robot->gamma_stepper_motor;

    this->should_log = this->enabled = THEKERNEL->config->value( touchprobe_log_enable_checksum )->by_default(false)->as_bool();
    if( this->should_log){
        this->filename = THEKERNEL->config->value(touchprobe_logfile_name_checksum)->by_default("/sd/probe_log.csv")->as_string();
        this->mcode = THEKERNEL->config->value(touchprobe_log_rotate_mcode_checksum)->by_default(0)->as_int();
        this->logfile = NULL;
    }
}

void Touchprobe::wait_for_touch(int distance[]){
    unsigned int debounce = 0;
    while(true){
        THEKERNEL->call_event(ON_IDLE);
        // if no stepper is moving, moves are finished and there was no touch
        if( ((this->steppers[0]->moving ? 0:1 ) + (this->steppers[1]->moving ? 0:1 ) + (this->steppers[2]->moving ? 0:1 )) == 3 ){
            return;
        }
        // if the touchprobe is active...
        if( this->pin.get() ){
            //...increase debounce counter...
            if( debounce < debounce_count) {
                // ...but only if the counter hasn't reached the max. value
                debounce++;
            } else {
                // ...otherwise stop the steppers, return its remaining steps
                for( int i=0; i<3; i++ ){
                    distance[i] = 0;
                    if ( this->steppers[i]->moving ){
                        distance[i] =  this->steppers[i]->stepped;
                        distance[i] *= this->steppers[i]->direction ? -1 : 1;
                        this->steppers[i]->move(0,0);
                    }
                }
                return;
            }
        }else{
            // The probe was not hit yet, reset debounce counter
            debounce = 0;
        }
    }
}


void Touchprobe::flush_log(){
    //FIXME *sigh* fflush doesn't work as expected, see: http://mbed.org/forum/mbed/topic/3234/ or http://mbed.org/search/?type=&q=fflush
    //fflush(logfile);
    fclose(logfile);
    //can't reopen the file here -> crash
    logfile = NULL;
}
// Workaround for the close<->reopen crash, which itself is a workaround for wrong (or unimplemented) fflush behaviour
void Touchprobe::on_idle(void* argument){
    if( this->logfile == NULL) {
        // NOTE: File creation is buggy, a file may appear but writing to it will fail
        this->logfile = fopen( filename.c_str(), "a");
    }
}

void Touchprobe::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);
    Robot* robot = THEKERNEL->robot;

    if( gcode->has_g) {
        if( gcode->g == 31 ) {
            float tmp[3], pos[3], mm[3];
            int steps[3];
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            robot->get_axis_position(pos);
            for(char c = 'X'; c <= 'Z'; c++){
                if( gcode->has_letter(c) ){
                    tmp[c-'X'] = robot->to_millimeters(gcode->get_value(c)) - ( robot->absolute_mode ? pos[c-'X'] : 0 );
                }else{
                    tmp[c-'X'] = 0;
                }
            }
            if( gcode->has_letter('F') )            {
                this->probe_rate = robot->to_millimeters( gcode->get_value('F') ) / robot->seconds_per_minute;
            }
            robot->arm_solution->cartesian_to_actuator(tmp, mm);
            for (int c = 0; c < 3; c++)
                steps[c] = mm[c] * robot->actuators[c]->steps_per_mm;

            if( ((abs(steps[0]) > 0 ? 1:0) + (abs(steps[1]) > 0 ? 1:0) + (abs(steps[2]) > 0 ? 1:0)) != 1 ){
                return; //TODO coordinated movement not supported yet
            }

            // Enable the motors
            THEKERNEL->stepper->turn_enable_pins_on();
            // move
            for(char c='X'; c<='Z'; c++){
                tmp[c - 'X'] = robot->actuators[c - 'X']->steps_per_mm;
                if( steps[c-'X'] == 0 ){
                    continue;
                }
                bool dir = steps[c-'X'] < 0;
                // tmp is steps/mm, probe_rate in mm/s -> speed needs steps/s
                this->steppers[c-'X']->set_speed(this->probe_rate * robot->actuators[c]->steps_per_mm);
                this->steppers[c-'X']->move(dir,abs(steps[c-'X']));
            }

            wait_for_touch(steps);
            // calculate new position
            for(char c='X'; c<='Z'; c++){
                robot->reset_axis_position(pos[c-'X'] + mm[c-'X'], c-'X');
            }

            if( this->should_log ){
                robot->get_axis_position(pos);
                fprintf(logfile,"%1.3f %1.3f %1.3f\n", robot->from_millimeters(pos[0]), robot->from_millimeters(pos[1]), robot->from_millimeters(pos[2]) );
                flush_log();
            }
        }
    }else if(gcode->has_m) {
        // log rotation
        // for now this only writes a separator
        // TODO do a actual log rotation
        if( this->mcode != 0 && this->should_log && gcode->m == this->mcode){
            string name;
            fputs("--\n",logfile);
            flush_log();
        }
    }
}

