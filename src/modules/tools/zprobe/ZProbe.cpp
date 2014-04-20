/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ZProbe.h"

#include "Kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "Stepper.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"
#include "Planner.h"

#define zprobe_checksum          CHECKSUM("zprobe")
#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define debounce_count_checksum  CHECKSUM("debounce_count")
#define feedrate_checksum        CHECKSUM("feedrate")

#define alpha_steps_per_mm_checksum      CHECKSUM("alpha_steps_per_mm")
#define beta_steps_per_mm_checksum       CHECKSUM("beta_steps_per_mm")
#define gamma_steps_per_mm_checksum      CHECKSUM("gamma_steps_per_mm")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

void ZProbe::on_module_loaded()
{
    // if the module is disabled -> do nothing
    this->enabled = THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(false)->as_bool();
    if( !(this->enabled) ) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    this->running= false;

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_CONFIG_RELOAD);
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_IDLE);

    THEKERNEL->slow_ticker->attach( THEKERNEL->stepper->acceleration_ticks_per_second , this, &ZProbe::acceleration_tick );
}

void ZProbe::on_config_reload(void *argument)
{
    this->pin.from_string(  THEKERNEL->config->value(zprobe_checksum, probe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->debounce_count =  THEKERNEL->config->value(zprobe_checksum, debounce_count_checksum)->by_default(0  )->as_number();

    this->steppers[0] = THEKERNEL->robot->alpha_stepper_motor;
    this->steppers[1] = THEKERNEL->robot->beta_stepper_motor;
    this->steppers[2] = THEKERNEL->robot->gamma_stepper_motor;

    // we need to know steps per mm
    this->steps_per_mm[0] =  THEKERNEL->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->steps_per_mm[1] =  THEKERNEL->config->value(beta_steps_per_mm_checksum)->as_number();
    this->steps_per_mm[2] =  THEKERNEL->config->value(gamma_steps_per_mm_checksum)->as_number();

    this->feedrate = THEKERNEL->config->value(zprobe_checksum, feedrate_checksum)->by_default(5)->as_number()*this->steps_per_mm[Z_AXIS]; // feedrate in steps/sec
}

bool ZProbe::wait_for_probe(int steps[])
{
    unsigned int debounce = 0;
    while(true) {
        THEKERNEL->call_event(ON_IDLE);
        // if no stepper is moving, moves are finished and there was no touch
        if( !this->steppers[X_AXIS]->moving && !this->steppers[Y_AXIS]->moving && !this->steppers[Z_AXIS]->moving ) {
            return false;
        }

        // if the touchprobe is active...
        if( this->pin.get() ) {
            //...increase debounce counter...
            if( debounce < debounce_count) {
                // ...but only if the counter hasn't reached the max. value
                debounce++;
            } else {
                // ...otherwise stop the steppers, return its remaining steps
                for( int i = X_AXIS; i <= Z_AXIS; i++ ) {
                    steps[i] = 0;
                    if ( this->steppers[i]->moving ) {
                        steps[i] =  this->steppers[i]->stepped;
                        this->steppers[i]->move(0, 0);
                    }
                }
                return true;
            }
        } else {
            // The probe was not hit yet, reset debounce counter
            debounce = 0;
        }
    }
}

void ZProbe::on_idle(void *argument)
{
}

// single probe and report amount moved
bool ZProbe::run_probe(int *steps)
{
    // Enable the motors
    THEKERNEL->stepper->turn_enable_pins_on();

    // move Z down
    // TODO for delta need to move all three actuators
    this->running= true;
    this->steppers[Z_AXIS]->set_speed(0); // will be increased by acceleration tick
    this->steppers[Z_AXIS]->move(true, 100*this->steps_per_mm[Z_AXIS]); // always probes down, no more than 100mm
    bool r= wait_for_probe(steps);
    this->running= false;
    return r;
}

void ZProbe::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    Robot *robot = THEKERNEL->robot;

    if( gcode->has_g) {
        // G code processing
        if( gcode->g == 30 ) {
            gcode->mark_as_taken();
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            if( gcode->has_letter('F') ) {
                // probe speed
                this->feedrate = robot->to_millimeters( gcode->get_value('F') ) * this->steps_per_mm[Z_AXIS] / robot->seconds_per_minute;
            }
            int steps[3];
            if(run_probe(steps)){
                gcode->stream->printf("Z:%1.4f C:%d\n", steps[2]/this->steps_per_mm[Z_AXIS], steps[Z_AXIS]);
                // move back to where it started, unless a Z is specified
                if(gcode->has_letter('Z')) {
                    // set Z to the specified value, and leave probe where it is
                    THEKERNEL->robot->reset_axis_position(gcode->get_value('Z'), Z_AXIS);
                }else{
                    // move probe back to where it was
                    this->steppers[Z_AXIS]->set_speed(0); // will be increased by acceleration tick
                    this->steppers[Z_AXIS]->move(false, steps[Z_AXIS]);
                    this->running= true;
                    while(this->steppers[Z_AXIS]->moving) { // wait for it to complete
                        THEKERNEL->call_event(ON_IDLE);
                    }
                    this->running= true;
                }
            }else{
                gcode->stream->printf("ZProbe not triggered\n");
            }
        }

    } else if(gcode->has_m) {
        // M code processing here
        if(gcode->m == 119) {
            int c= this->pin.get();
            gcode->stream->printf(" Probe: %d", c);
            gcode->add_nl = true;
            gcode->mark_as_taken();
        }
    }
}

#define max(a,b) (((a) > (b)) ? (a) : (b))
// Called periodically to change the speed to match acceleration
uint32_t ZProbe::acceleration_tick(uint32_t dummy)
{
    if(!this->running) return(0); // nothing to do

    // foreach stepper that is moving
    for ( int c = 0; c <= 2; c++ ) {
        if( !this->steppers[c]->moving ) continue;

        uint32_t current_rate = this->steppers[c]->steps_per_second;
        uint32_t target_rate = int(floor(this->feedrate));

        if( current_rate < target_rate ){
            uint32_t rate_increase = int(floor((THEKERNEL->planner->acceleration/THEKERNEL->stepper->acceleration_ticks_per_second)*this->steps_per_mm[c]));
            current_rate = min( target_rate, current_rate + rate_increase );
        }
        if( current_rate > target_rate ){ current_rate = target_rate; }

        // steps per second
        this->steppers[c]->set_speed(max(current_rate, THEKERNEL->stepper->minimum_steps_per_second));
    }

    return 0;
}
