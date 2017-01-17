
#include "Jogger.h"

#include <math.h>
#include "Kernel.h"
#include "Robot.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"

#include "JoystickPublicAccess.h"
#include "PublicData.h"
#include "utils.h"

#include "StreamOutputPool.h" //just for debugging

#define jogger_checksum                     CHECKSUM("jogger")
#define enable_checksum                     CHECKSUM("enable")
#define default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define max_speed_checksum                  CHECKSUM("max_speed")
#define dead_zone_checksum                  CHECKSUM("dead_zone")
#define nonlinearity_checksum               CHECKSUM("nonlinearity")
#define refresh_rate_checksum               CHECKSUM("refresh_rate")
#define step_scale_factor_checksum          CHECKSUM("step_scale_factor")

#define axis0_data_source_checksum          CHECKSUM("data_source_alpha")
#define axis1_data_source_checksum          CHECKSUM("data_source_beta")
#define axis2_data_source_checksum          CHECKSUM("data_source_gamma")
#define axis3_data_source_checksum          CHECKSUM("data_source_delta")
#define axis4_data_source_checksum          CHECKSUM("data_source_epsilon")
#define axis5_data_source_checksum          CHECKSUM("data_source_zeta")

#define abs(a) ((a<0) ? -a : a)
#define sign(a) ((a<0) ? -1 : 1)

Jogger::Jogger() {}

//deconstructor: THEKERNEL->unregister_for_event(ON_GCODE_RECEIVED, this);
//add config for which M-code(s) to respond to

void Jogger::on_module_loaded()
{
    if (!THEKERNEL->config->value(jogger_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as not needed free up resource
        delete this;
        return;
    }

    //load configuration for this module
    this->on_config_reload(this);

    //register for events with the kernel
    this->register_for_event(ON_GCODE_RECEIVED);

    //ask the kernel to run "update_tick" every "refresh_interval" milliseconds
    THEKERNEL->slow_ticker->attach(this->refresh_rate, this, &Jogger::update_tick);

}

//debug only
void Jogger::on_gcode_received(void *argument)
{
    //testing code here
    //print out parameters
    THEKERNEL->streams->printf("%+0.2f, %+0.2f      Max: %0.1f, Dead: %f, Nl: %f, Rate: %d\n", this->position[0], this->position[1], max_speed, dead_zone, nonlinearity, refresh_rate);
}

//read config file values for this module
void Jogger::on_config_reload(void *argument)
{
    this->max_speed = THEKERNEL->config->value(default_seek_rate_checksum)->by_default(this->max_speed)->as_number();
    this->max_speed = THEKERNEL->config->value(joystick_checksum, max_speed_checksum)->by_default(this->max_speed)->as_number();
    this->dead_zone = THEKERNEL->config->value(joystick_checksum, dead_zone_checksum)->by_default(this->dead_zone)->as_number();
    this->nonlinearity = THEKERNEL->config->value(joystick_checksum, nonlinearity_checksum)->by_default(this->nonlinearity)->as_number();
    this->refresh_rate = THEKERNEL->config->value(jogger_checksum, refresh_rate_checksum)->by_default(this->refresh_rate)->as_number();
    this->step_scale_factor = THEKERNEL->config->value(jogger_checksum, step_scale_factor_checksum)->by_default(this->step_scale_factor)->as_number();
    
    //load the names of the joystick modules where each axis will get its data
    uint16_t axisN_data_source_checksum[] = { axis0_data_source_checksum, axis1_data_source_checksum, axis2_data_source_checksum, axis3_data_source_checksum, axis4_data_source_checksum, axis5_data_source_checksum };
    for (int i = 0; i < NUM_JOG_AXES; i++) {
        this->axis_data_source[i] = get_checksum(THEKERNEL->config->value(jogger_checksum, axisN_data_source_checksum[i])->by_default("")->as_string());
    }
    
}

//map a position from -1 to 1 into a speed from -max to max
float Jogger::get_speed(float pos) {
    float speed;

    //apply non-linear scaling to position
    speed = (sign(pos) * pow(abs(pos), nonlinearity)) * max_speed;

    //zero out in dead zone
    if (abs(pos) <= dead_zone) {
        speed = 0.0f;
    }

    return speed;
}

//runs on a timer to update the jog speeds
uint32_t Jogger::update_tick(uint32_t dummy)
{

    //read the joysticks for each axis and map their values to a target speed
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        struct PAD_joystick s;
        if (PublicData::get_value(joystick_checksum, this->axis_data_source[c], &s)) {
            this->position[c] = s.position;
        }
        else {
            this->position[c] = 0;
            //position should be 0 if the module doesn't respond, or the module name is unspecified/wrong
            //no movement on that axis if position is 0 = safe
        }
    }

    //update the target speeds given the positions
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        this->target_speed[c] = get_speed(this->position[c]);
    }

    //METHOD 2: ask robot for small change in position with desired speed, and do so when the robot has nothing else to do
    if (THECONVEYOR->is_queue_empty()) {
        //if the queue has been emptied, or is idle, it is safe to do a jog

        //NOTE: getting the machine's position doesn't include the inverse compensation transform
        //until the compensation transforms are more like the arm solutions and have both forward and inverse solutions,
        //jogging will take place in the machine's coordinate system (i.e. jogging X/Y won't change Z-axis even with a bed-leveling compensation enabled)

        //push the current robot state (save G90/91 setting)
        THEROBOT->push_state();

        //set the robot's mode to relative motion
        THEROBOT->absolute_mode = false;

        //create a new gcode to move a small distance in the direction given by the joystick, at the speed defined by the joystick position
        char command[64];
        const float sf = 1.0f; //scale -1 to 1 position to mm per small move

        int n = snprintf(command, sizeof(command), "G0 X%0.3f Y%0.3f F%0.1f", this->position[0] * sf, this->position[1] * sf, sqrt(pow(this->target_speed[0], 2) + pow(this->target_speed[1], 2)));

        Gcode gc(command, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

        //pop the saved robot state (return to whatever G90/91 setting was before)
        THEROBOT->pop_state();

        THEKERNEL->streams->printf(">>> %s\n", command);

        /*
        // current actuator position in mm
        float axis_pos[NUM_JOG_AXES];
        THEROBOT->get_axis_position(axis_pos, NUM_JOG_AXES);
        // get machine position from the actuator position using FK
        float machine_pos[3];
        THEROBOT->arm_solution->actuator_to_cartesian(axis_pos, machine_pos);
        */
    }

    /*
    //update stepper's target speed
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        this->target_speed[c] = get_speed(this->position[c]) * STEPS_PER_MM(c); //TODO: figure out which axis we are referring to

        //stop here if any endstop safety checks fail (only checking in direction of travel)
        if (this->endstop_pin[c + ((target_feedrate[c]<0) ? 0 : 3)].get()) continue;

        //enable the stepper if the target speed is non-zero
        if (!this->enabled[c] && target_speed[c] != 0.0f) {
            this->enabled[c] = true;
            THEKERNEL->stepper->turn_enable_pins_on();
        }
    }

    //if jogging, update the robot's last milestone (i.e. position) to the steppers' current positions
    THEKERNEL->robot->reset_position_from_current_actuator_position(); //TODO: determine if this is the correct way of updating position of robot
    */

    return 0;
}


/*TODO: somehow attach this function for accel calculations, no longer possible through acceleration_tick_handler?
// Called periodically to change the speed to match acceleration
void Jogger::acceleration_tick(void)
{
    //update enabled steppers' speeds
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        if (enabled[c]) accelerate(c);
    }

    return;
}

//acceleration calculation, just need to set this->current_feedrate
void Jogger::accelerate(int c)
{
    float current_rate = (direction[c]) ? STEPPER[c]->get_steps_per_second() : -1 * (STEPPER[c]->get_steps_per_second());
    if (!STEPPER[c]->is_moving()) current_rate = 0;

    float target_rate = this->target_speed[c];
    float old_rate = current_rate;

    // Z may have a different acceleration to X and Y
    float acc = (c == Z_AXIS) ? THEKERNEL->planner->get_z_acceleration() : THEKERNEL->planner->get_acceleration();
    float rate_change = (acc / THEKERNEL->acceleration_ticks_per_second) * STEPS_PER_MM(c);

    //update the current rate using the acceleration
    if (current_rate <= target_rate) {
        current_rate = min(target_rate, current_rate + rate_change);
    }
    else {
        current_rate = max(target_rate, current_rate - rate_change);
    }

    //check if current rate is zero, stop motors if so
    if (current_rate == 0) {
        STEPPER[c]->move(0, 0);
        enabled[c] = false;

    }
    else {
        //check if switched sign from last speed to new speed, or last speed was 0
        if (sign(old_rate) != sign(current_rate) || old_rate == 0) {
            //update direction
            direction[c] = current_rate>0;

            //then command a motor movement
            // not a block move so disable the last tick setting? (not sure about this line, copied from example)
            STEPPER[c]->set_moved_last_block(false);

            //start motor moving (commanded distance is 100 meters, should be large enough to ensure the motor won't stop unexpectedly)
            STEPPER[c]->move(direction[c], 100000 * STEPS_PER_MM(c), 0);
        }

        //set motor steps per second
        STEPPER[c]->set_speed(abs(current_rate));
    }
}
*/