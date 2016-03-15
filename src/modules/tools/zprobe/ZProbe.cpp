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
#include "utils.h"

// strategies we know about
#include "DeltaCalibrationStrategy.h"
#include "ThreePointStrategy.h"
#include "ZGridStrategy.h"
#include "DeltaGridStrategy.h"

#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define debounce_count_checksum  CHECKSUM("debounce_count")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")
#define return_feedrate_checksum CHECKSUM("return_feedrate")
#define probe_height_checksum    CHECKSUM("probe_height")
#define gamma_max_checksum       CHECKSUM("gamma_max")
#define reverse_z_direction_checksum CHECKSUM("reverse_z")

// from endstop section
#define delta_homing_checksum    CHECKSUM("delta_homing")
#define rdelta_homing_checksum    CHECKSUM("rdelta_homing")

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

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);

    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){acceleration_tick(); });

    // we read the probe in this timer, currently only for G38 probes.
    probing= false;
    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_probe);
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

                case delta_grid_leveling_strategy_checksum:
                    this->strategies.push_back(new DeltaGridStrategy(this));
                    found= true;
                    break;
            }
            if(found) this->strategies.back()->handleConfig();
        }
    }

    // need to know if we need to use delta kinematics for homing
    this->is_delta = THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_rdelta = THEKERNEL->config->value(rdelta_homing_checksum)->by_default(false)->as_bool();

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
    this->return_feedrate = THEKERNEL->config->value(zprobe_checksum, return_feedrate_checksum)->by_default(0)->as_number(); // feedrate in mm/sec
    this->reverse_z     = THEKERNEL->config->value(zprobe_checksum, reverse_z_direction_checksum)->by_default(false)->as_bool(); // Z probe moves in reverse direction
    this->max_z         = THEKERNEL->config->value(gamma_max_checksum)->by_default(500)->as_number(); // maximum zprobe distance
}

bool ZProbe::wait_for_probe(int& steps)
{
    unsigned int debounce = 0;
    while(true) {
        THEKERNEL->call_event(ON_IDLE);
        if(THEKERNEL->is_halted()){
            // aborted by kill
            return false;
        }

        bool delta= is_delta || is_rdelta;

        // if no stepper is moving, moves are finished and there was no touch
        if( !STEPPER[Z_AXIS]->is_moving() && (!delta || (!STEPPER[Y_AXIS]->is_moving() && !STEPPER[Z_AXIS]->is_moving())) ) {
            return false;
        }

        // if the probe is active...
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
                if(delta) {
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

// single probe with custom feedrate
// returns boolean value indicating if probe was triggered
bool ZProbe::run_probe(int& steps, float feedrate, float max_dist, bool reverse)
{
    // not a block move so disable the last tick setting
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        STEPPER[c]->set_moved_last_block(false);
    }

    // Enable the motors
    THEKERNEL->stepper->turn_enable_pins_on();
    this->current_feedrate = feedrate * Z_STEPS_PER_MM; // steps/sec
    float maxz= max_dist < 0 ? this->max_z*2 : max_dist;

    // move Z down
    bool dir= (!reverse_z != reverse); // xor
    STEPPER[Z_AXIS]->move(dir, maxz * Z_STEPS_PER_MM, 0); // probe in specified direction, no more than maxz
    if(this->is_delta || this->is_rdelta) {
        // for delta need to move all three actuators
        STEPPER[X_AXIS]->move(dir, maxz * STEPS_PER_MM(X_AXIS), 0);
        STEPPER[Y_AXIS]->move(dir, maxz * STEPS_PER_MM(Y_AXIS), 0);
    }

    // start acceleration processing
    this->running = true;

    bool r = wait_for_probe(steps);
    this->running = false;
    STEPPER[X_AXIS]->move(0, 0);
    STEPPER[Y_AXIS]->move(0, 0);
    STEPPER[Z_AXIS]->move(0, 0);
    return r;
}

bool ZProbe::return_probe(int steps, bool reverse)
{
    // move probe back to where it was

    float fr;
    if(this->return_feedrate != 0) { // use return_feedrate if set
        fr = this->return_feedrate;
    } else {
        fr = this->slow_feedrate*2; // nominally twice slow feedrate
        if(fr > this->fast_feedrate) fr = this->fast_feedrate; // unless that is greater than fast feedrate
    }

    this->current_feedrate = fr * Z_STEPS_PER_MM; // feedrate in steps/sec
    bool dir= ((steps < 0) != reverse_z); // xor

    if(reverse) dir= !dir;
    steps= abs(steps);

    bool delta= (this->is_delta || this->is_rdelta);
    STEPPER[Z_AXIS]->move(dir, steps, 0);
    if(delta) {
        STEPPER[X_AXIS]->move(dir, steps, 0);
        STEPPER[Y_AXIS]->move(dir, steps, 0);
    }

    this->running = true;
    while(STEPPER[Z_AXIS]->is_moving() || (delta && (STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving())) ) {
        // wait for it to complete
        THEKERNEL->call_event(ON_IDLE);
         if(THEKERNEL->is_halted()){
            // aborted by kill
            break;
        }
    }

    this->running = false;
    STEPPER[X_AXIS]->move(0, 0);
    STEPPER[Y_AXIS]->move(0, 0);
    STEPPER[Z_AXIS]->move(0, 0);

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
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            int steps;
            bool probe_result;
            bool reverse= (gcode->has_letter('R') && gcode->get_value('R') != 0); // specify to probe in reverse direction
            float rate= gcode->has_letter('F') ? gcode->get_value('F') / 60 : this->slow_feedrate;
            probe_result = run_probe(steps, rate, -1, reverse);

            if(probe_result) {
                // the result is in actuator coordinates and raw steps
                gcode->stream->printf("Z:%1.4f C:%d\n", zsteps_to_mm(steps), steps);

                // set the last probe position to the current actuator units
                THEKERNEL->robot->set_last_probe_position(std::make_tuple(
                    THEKERNEL->robot->actuators[X_AXIS]->get_current_position(),
                    THEKERNEL->robot->actuators[Y_AXIS]->get_current_position(),
                    THEKERNEL->robot->actuators[Z_AXIS]->get_current_position(),
                    1));

                // move back to where it started, unless a Z is specified (and not a rotary delta)
                if(gcode->has_letter('Z') && !is_rdelta) {
                    // set Z to the specified value, and leave probe where it is
                    THEKERNEL->robot->reset_axis_position(gcode->get_value('Z'), Z_AXIS);

                } else {
                    // return to pre probe position
                    return_probe(steps, reverse);
                }

            } else {
                gcode->stream->printf("ZProbe not triggered\n");
                THEKERNEL->robot->set_last_probe_position(std::make_tuple(
                    THEKERNEL->robot->actuators[X_AXIS]->get_current_position(),
                    THEKERNEL->robot->actuators[Y_AXIS]->get_current_position(),
                    THEKERNEL->robot->actuators[Z_AXIS]->get_current_position(),
                    0));
            }

        } else {
            if(!gcode->has_letter('P')) {
                // find the first strategy to handle the gcode
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
                gcode->stream->printf("No strategy found to handle G%d\n", gcode->g);

            }else{
                // P paramater selects which strategy to send the code to
                // they are loaded in the order they are defined in config, 0 being the first, 1 being the second and so on.
                uint16_t i= gcode->get_value('P');
                if(i < strategies.size()) {
                    if(!strategies[i]->handleGcode(gcode)){
                        gcode->stream->printf("strategy #%d did not handle G%d\n", i, gcode->g);
                    }
                    return;

                }else{
                    gcode->stream->printf("strategy #%d is not loaded\n", i);
                }
            }
        }

    } else if(gcode->has_g && gcode->g == 38 ) { // G38.2 Straight Probe with error, G38.3 straight probe without error
        // linuxcnc/grbl style probe http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G38-probe
        if(gcode->subcode != 2 && gcode->subcode != 3) {
            gcode->stream->printf("error:Only G38.2 and G38.3 are supported\n");
            return;
        }

        // make sure the probe is defined and not already triggered before moving motors
        if(!this->pin.connected()) {
            gcode->stream->printf("error:ZProbe not connected.\n");
            return;
        }

        if(this->pin.get()) {
            gcode->stream->printf("error:ZProbe triggered before move, aborting command.\n");
            return;
        }

        // first wait for an empty queue i.e. no moves left
        THEKERNEL->conveyor->wait_for_empty_queue();

        // turn off any compensation transform
        auto savect= THEKERNEL->robot->compensationTransform;
        THEKERNEL->robot->compensationTransform= nullptr;

        if(gcode->has_letter('X')) {
            // probe in the X axis
            probe_XYZ(gcode, X_AXIS);

        }else if(gcode->has_letter('Y')) {
            // probe in the Y axis
            probe_XYZ(gcode, Y_AXIS);

        }else if(gcode->has_letter('Z')) {
            // probe in the Z axis
            probe_XYZ(gcode, Z_AXIS);

        }else{
            gcode->stream->printf("error:at least one of X Y or Z must be specified\n");
        }

        // restore compensationTransform
        THEKERNEL->robot->compensationTransform= savect;

        return;

    } else if(gcode->has_m) {
        // M code processing here
        int c;
        switch (gcode->m) {
            case 119:
                c = this->pin.get();
                gcode->stream->printf(" Probe: %d", c);
                gcode->add_nl = true;
                break;

            case 670:
                if (gcode->has_letter('S')) this->slow_feedrate = gcode->get_value('S');
                if (gcode->has_letter('K')) this->fast_feedrate = gcode->get_value('K');
                if (gcode->has_letter('R')) this->return_feedrate = gcode->get_value('R');
                if (gcode->has_letter('Z')) this->max_z = gcode->get_value('Z');
                if (gcode->has_letter('H')) this->probe_height = gcode->get_value('H');
                if (gcode->has_letter('I')) { // NOTE this is temporary and toggles the invertion status of the pin
                    invert_override= (gcode->get_value('I') != 0);
                    pin.set_inverting(pin.is_inverting() != invert_override); // XOR so inverted pin is not inverted and vice versa
                }
                break;

            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Probe feedrates Slow/fast(K)/Return (mm/sec) max_z (mm) height (mm):\nM670 S%1.2f K%1.2f R%1.2f Z%1.2f H%1.2f\n",
                    this->slow_feedrate, this->fast_feedrate, this->return_feedrate, this->max_z, this->probe_height);

                // fall through is intended so leveling strategies can handle m-codes too

            default:
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
        }
    }
}

uint32_t ZProbe::read_probe(uint32_t dummy)
{
    if(!probing || probe_detected) return 0;

    // TODO add debounce/noise filter
    if(this->pin.get()) {
        probe_detected= true;
        // now tell all the stepper_motors to stop
        for(auto &a : THEKERNEL->robot->actuators) a->force_finish_move();
    }
    return 0;
}

// special way to probe in the X or Y or Z direction using planned moves, should work with any kinematics
void ZProbe::probe_XYZ(Gcode *gcode, int axis)
{
    // enable the probe checking in the timer
    probing= true;
    probe_detected= false;
    THEKERNEL->robot->disable_segmentation= true; // we must disable segmentation as this won't work with it enabled (beware on deltas probing in X or Y)

    // get probe feedrate if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F')*60 : this->slow_feedrate;

    // do a regular move which will stop as soon as the probe is triggered, or the distance is reached
    switch(axis) {
        case X_AXIS: coordinated_move(gcode->get_value('X'), 0, 0, rate, true); break;
        case Y_AXIS: coordinated_move(0, gcode->get_value('Y'), 0, rate, true); break;
        case Z_AXIS: coordinated_move(0, 0, gcode->get_value('Z'), rate, true); break;
    }

    // coordinated_move returns when the move is finished

    // disable probe checking
    probing= false;
    THEKERNEL->robot->disable_segmentation= false;

    float pos[3];
    {
        // get the current position
        ActuatorCoordinates current_position{
            THEKERNEL->robot->actuators[X_AXIS]->get_current_position(),
            THEKERNEL->robot->actuators[Y_AXIS]->get_current_position(),
            THEKERNEL->robot->actuators[Z_AXIS]->get_current_position()
        };

        // get machine position from the actuator position using FK
        THEKERNEL->robot->arm_solution->actuator_to_cartesian(current_position, pos);
    }

    uint8_t probeok= this->probe_detected ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], probeok);
    THEKERNEL->robot->set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], probeok));

    if(!probeok && gcode->subcode == 2) {
        // issue error if probe was not triggered and subcode == 2
        gcode->stream->printf("ALARM:Probe fail\n");
        THEKERNEL->call_event(ON_HALT, nullptr);

    }else if(probeok){
        // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
        THEKERNEL->robot->reset_position_from_current_actuator_position();
    }
}

// Called periodically to change the speed to match acceleration
void ZProbe::acceleration_tick(void)
{
    if(!this->running) return; // nothing to do
    if(STEPPER[Z_AXIS]->is_moving()) accelerate(Z_AXIS);

    if(is_delta || is_rdelta) {
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
// NOTE must use G53 to force move in machine coordiantes and ignore any WCS offsetts
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative)
{
    char buf[32];
    char cmd[64];

    if(relative) strcpy(cmd, "G91 G0 ");
    else strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system

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
