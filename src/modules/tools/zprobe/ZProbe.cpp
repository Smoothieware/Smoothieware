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
#include "SerialMessage.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include "LevelingStrategy.h"
#include "StepTicker.h"

// strategies we know about
#include "DeltaCalibrationStrategy.h"
#include "ThreePointStrategy.h"
#include "ZGridStrategy.h"

#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define debounce_count_checksum  CHECKSUM("debounce_count")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")
#define probe_height_checksum    CHECKSUM("probe_height")
#define gamma_max_checksum       CHECKSUM("gamma_max")

// from endstop section
#define delta_homing_checksum    CHECKSUM("delta_homing")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

#define abs(a) ((a<0) ? -a : a)

void ZProbe::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    this->running = false;

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);

    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){acceleration_tick(); });
}

void ZProbe::on_config_reload(void *argument)
{
    this->pin.from_string( THEKERNEL->config->value(zprobe_checksum, probe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->debounce_count = THEKERNEL->config->value(zprobe_checksum, debounce_count_checksum)->by_default(0  )->as_number();

    // get strategies to load
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, leveling_strategy_checksum);
    for( auto cs : modules ){
        if( THEKERNEL->config->value(leveling_strategy_checksum, cs, enable_checksum )->as_bool() ){
            bool found= false;
            // check with each known strategy and load it if it matches
            switch(cs) {
                case delta_calibration_strategy_checksum:
                    this->strategies.push_back(new DeltaCalibrationStrategy(this));
                    found= true;
                    break;

                case three_point_leveling_strategy_checksum:
                    // NOTE this strategy is mutually exclusive with the delta calibration strategy
                    this->strategies.push_back(new ThreePointStrategy(this));
                    found= true;
                    break;

                case ZGrid_leveling_checksum:
                     this->strategies.push_back(new ZGridStrategy(this));
                     found= true;
                     break;

                // add other strategies here
                //case zheight_map_strategy:
                //     this->strategies.push_back(new ZHeightMapStrategy(this));
                //     found= true;
                //     break;
            }
            if(found) this->strategies.back()->handleConfig();
        }
    }

    // need to know if we need to use delta kinematics for homing
    this->is_delta = THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();

    // default for backwards compatibility add DeltaCalibrationStrategy if a delta
    // will be deprecated
    if(this->strategies.empty()) {
        if(this->is_delta) {
            this->strategies.push_back(new DeltaCalibrationStrategy(this));
            this->strategies.back()->handleConfig();
        }
    }

    this->probe_height  = THEKERNEL->config->value(zprobe_checksum, probe_height_checksum)->by_default(5.0F)->as_number();
    this->slow_feedrate = THEKERNEL->config->value(zprobe_checksum, slow_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->fast_feedrate = THEKERNEL->config->value(zprobe_checksum, fast_feedrate_checksum)->by_default(100)->as_number(); // feedrate in mm/sec
    this->max_z         = THEKERNEL->config->value(gamma_max_checksum)->by_default(500)->as_number(); // maximum zprobe distance
}

bool ZProbe::wait_for_probe(int& steps)
{
    unsigned int debounce = 0;
    while(true) {
        THEKERNEL->call_event(ON_IDLE);
        // if no stepper is moving, moves are finished and there was no touch
        if( !STEPPER[Z_AXIS]->is_moving() && (!is_delta || (!STEPPER[Y_AXIS]->is_moving() && !STEPPER[Z_AXIS]->is_moving())) ) {
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
                if(STEPPER[Z_AXIS]->is_moving()){
                    steps= STEPPER[Z_AXIS]->get_stepped();
                    STEPPER[Z_AXIS]->move(0, 0);
                }
                if(is_delta) {
                    for( int i = X_AXIS; i <= Y_AXIS; i++ ) {
                        if ( STEPPER[i]->is_moving() ) {
                            STEPPER[i]->move(0, 0);
                        }
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

// single probe and report amount moved
bool ZProbe::run_probe(int& steps, bool fast)
{
    // not a block move so disable the last tick setting
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        STEPPER[c]->set_moved_last_block(false);
    }

    // Enable the motors
    THEKERNEL->stepper->turn_enable_pins_on();
    this->current_feedrate = (fast ? this->fast_feedrate : this->slow_feedrate) * Z_STEPS_PER_MM; // steps/sec
    float maxz= this->max_z*2;

    // move Z down
    STEPPER[Z_AXIS]->move(true, maxz * Z_STEPS_PER_MM, 0); // always probes down, no more than 2*maxz
    if(this->is_delta) {
        // for delta need to move all three actuators
        STEPPER[X_AXIS]->move(true, maxz * STEPS_PER_MM(X_AXIS), 0);
        STEPPER[Y_AXIS]->move(true, maxz * STEPS_PER_MM(Y_AXIS), 0);
    }

    // start acceleration processing
    this->running = true;

    bool r = wait_for_probe(steps);
    this->running = false;
    return r;
}

bool ZProbe::return_probe(int steps)
{
    // move probe back to where it was
    float fr= this->slow_feedrate*2; // nominally twice slow feedrate
    if(fr > this->fast_feedrate) fr= this->fast_feedrate; // unless that is greater than fast feedrate
    this->current_feedrate = fr * Z_STEPS_PER_MM; // feedrate in steps/sec
    bool dir= steps < 0;
    steps= abs(steps);

    STEPPER[Z_AXIS]->move(dir, steps, 0);
    if(this->is_delta) {
        STEPPER[X_AXIS]->move(dir, steps, 0);
        STEPPER[Y_AXIS]->move(dir, steps, 0);
    }

    this->running = true;
    while(STEPPER[Z_AXIS]->is_moving() || (is_delta && (STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving())) ) {
        // wait for it to complete
        THEKERNEL->call_event(ON_IDLE);
    }

    this->running = false;

    return true;
}

bool ZProbe::doProbeAt(int &steps, float x, float y)
{
    int s;
    // move to xy
    coordinated_move(x, y, NAN, getFastFeedrate());
    if(!run_probe(s)) return false;

    // return to original Z
    return_probe(s);
    steps = s;

    return true;
}

float ZProbe::probeDistance(float x, float y)
{
    int s;
    if(!doProbeAt(s, x, y)) return NAN;
    return zsteps_to_mm(s);
}

void ZProbe::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 32) {
        // make sure the probe is defined and not already triggered before moving motors
       if(!this->pin.connected()) {
            gcode->stream->printf("ZProbe not connected.\n");
            return;
        }
        if(this->pin.get()) {
            gcode->stream->printf("ZProbe triggered before move, aborting command.\n");
            return;
        }

        if( gcode->g == 30 ) { // simple Z probe
            gcode->mark_as_taken();
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            int steps;
            if(run_probe(steps)) {
                gcode->stream->printf("Z:%1.4f C:%d\n", steps / Z_STEPS_PER_MM, steps);
                // move back to where it started, unless a Z is specified
                if(gcode->has_letter('Z')) {
                    // set Z to the specified value, and leave probe where it is
                    THEKERNEL->robot->reset_axis_position(gcode->get_value('Z'), Z_AXIS);
                } else {
                    return_probe(steps);
                }
            } else {
                gcode->stream->printf("ZProbe not triggered\n");
            }

        } else {
            // find a strategy to handle the gcode
            for(auto s : strategies){
                if(s->handleGcode(gcode)) {
                    gcode->mark_as_taken();
                    return;
                }
            }
            gcode->stream->printf("No strategy found to handle G%d\n", gcode->g);
        }

    } else if(gcode->has_m) {
        // M code processing here
        if(gcode->m == 119) {
            int c = this->pin.get();
            gcode->stream->printf(" Probe: %d", c);
            gcode->add_nl = true;
            gcode->mark_as_taken();

        }else {
            for(auto s : strategies){
                if(s->handleGcode(gcode)) {
                    gcode->mark_as_taken();
                    return;
                }
            }
        }
    }
}

// Called periodically to change the speed to match acceleration
void ZProbe::acceleration_tick(void)
{
    if(!this->running) return; // nothing to do
    if(STEPPER[Z_AXIS]->is_moving()) accelerate(Z_AXIS);

    if(is_delta) {
         // deltas needs to move all actuators
        for ( int c = X_AXIS; c <= Y_AXIS; c++ ) {
            if( !STEPPER[c]->is_moving() ) continue;
            accelerate(c);
        }
    }

    return;
}

void ZProbe::accelerate(int c)
{   uint32_t current_rate = STEPPER[c]->get_steps_per_second();
    uint32_t target_rate = floorf(this->current_feedrate);

    // Z may have a different acceleration to X and Y
    float acc= (c==Z_AXIS) ? THEKERNEL->planner->get_z_acceleration() : THEKERNEL->planner->get_acceleration();
    if( current_rate < target_rate ) {
        uint32_t rate_increase = floorf((acc / THEKERNEL->acceleration_ticks_per_second) * STEPS_PER_MM(c));
        current_rate = min( target_rate, current_rate + rate_increase );
    }
    if( current_rate > target_rate ) {
        current_rate = target_rate;
    }

    // steps per second
    STEPPER[c]->set_speed(current_rate);
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative)
{
    char buf[32];
    char cmd[64];

    if(relative) strcpy(cmd, "G91 G0 ");
    else strcpy(cmd, "G0 ");

    if(!isnan(x)) {
        int n = snprintf(buf, sizeof(buf), " X%1.3f", x);
        strncat(cmd, buf, n);
    }
    if(!isnan(y)) {
        int n = snprintf(buf, sizeof(buf), " Y%1.3f", y);
        strncat(cmd, buf, n);
    }
    if(!isnan(z)) {
        int n = snprintf(buf, sizeof(buf), " Z%1.3f", z);
        strncat(cmd, buf, n);
    }

    // use specified feedrate (mm/sec)
    int n = snprintf(buf, sizeof(buf), " F%1.1f", feedrate * 60); // feed rate is converted to mm/min
    strncat(cmd, buf, n);
    if(relative) strcat(cmd, " G90");

    //THEKERNEL->streams->printf("DEBUG: move: %s\n", cmd);

    // send as a command line as may have multiple G codes in it
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_empty_queue();
}

// issue home command
void ZProbe::home()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

float ZProbe::zsteps_to_mm(float steps)
{
    return steps / Z_STEPS_PER_MM;
}
