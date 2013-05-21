/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Touchprobe.h"

void Touchprobe::on_module_loaded() {
	// if the module is disabled -> do nothing
    if( this->kernel->config->value( touchprobe_enable_checksum )->by_default(false)->as_bool() == false ){ return; }
    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_CONFIG_RELOAD);
    register_for_event(ON_GCODE_RECEIVED);
}

void Touchprobe::on_config_reload(void* argument){
    this->pin.from_string(  this->kernel->config->value(touchprobe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->debounce_count =  this->kernel->config->value(endstop_debounce_count_checksum      )->by_default(100  )->as_number();

    this->steppers[0] = this->kernel->robot->alpha_stepper_motor;
    this->steppers[1] = this->kernel->robot->beta_stepper_motor;
    this->steppers[2] = this->kernel->robot->gamma_stepper_motor;

}


void Touchprobe::wait_for_touch(int remaining_steps[]){
    bool running = true;
    unsigned int debounce = 0;

    while(running){
        running = false;
        this->kernel->call_event(ON_IDLE);
        // if the touchprobe is active or there are no moves left
        //     -> the probe only hit air, move fully completed...
        if( this->pin.get() || (this->kernel->conveyor->queue.size() == 0 ) ){
            //...increase debounce counter...
            if( debounce < debounce_count) {
                // ...but only if the counter hasn't reached the max. value
                debounce++;
                running = true;
            } else {
                // ...otherwise stop the steppers, return it's remaining steps
                for( int i=0; i<3; i++ ){
                    if ( this->steppers[i]->moving ){
                        remaining_steps[i] =  this->steppers[i]->steps_to_move - this->steppers[i]->stepped;
                        remaining_steps[i] *= this->steppers[i]->dir_pin->get() ? -1 : 1;
                        this->steppers[i]->move(0,0);
                    }
                }
            }
        }else{
            // The probe was not hit yet
            running = true;
            debounce = 0;
        }
    }
    this->kernel->streams->printf( "touch end\r\n" );
}

void Touchprobe::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);

    if( gcode->has_g)
    {
        if( gcode->g == 31 )
        {
            int remaining_steps[3];
            string s;
            Gcode g1_copy(*gcode);

            // First wait for the queue to be empty
            this->kernel->conveyor->wait_for_empty_queue();
            // send a fake G1 with the same arguments to the robot
            s = "G1 " + gcode->command.substr(3);
            g1_copy = Gcode(s,gcode->stream);
            gcode->stream->printf("code '%s'\n", g1_copy.command.c_str() );
            this->kernel->robot->on_gcode_received((void*)&g1_copy);
            gcode->stream->printf("Pin: %d.%d =%d ", this->pin.port_number, this->pin.pin, this->pin.get() );

            wait_for_touch(remaining_steps);

            //clear the queue
            while( this->kernel->conveyor->queue.size() > 0 ) {
                this->kernel->conveyor->queue.delete_first();
            }
        }
    }
}
