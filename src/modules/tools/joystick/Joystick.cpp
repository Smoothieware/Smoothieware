#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "Joystick.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Stepper.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "StepTicker.h"

#define joystick_checksum                   CHECKSUM("joystick")
#define enable_checksum                     CHECKSUM("enable")
#define horizontal_pin_checksum             CHECKSUM("horizontal_pin")
#define vertical_pin_checksum               CHECKSUM("vertical_pin")
#define default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define max_speed_checksum                  CHECKSUM("max_speed")
#define dead_zone_checksum                  CHECKSUM("dead_zone")
#define zero_offset_horizontal_checksum     CHECKSUM("zero_offset_horizontal")
#define zero_offset_vertical_checksum       CHECKSUM("zero_offset_vertical")
#define nonlinearity_checksum               CHECKSUM("nonlinearity")

#define alpha_min_endstop_checksum       CHECKSUM("alpha_min_endstop")
#define beta_min_endstop_checksum        CHECKSUM("beta_min_endstop")
#define gamma_min_endstop_checksum       CHECKSUM("gamma_min_endstop")
#define alpha_max_endstop_checksum       CHECKSUM("alpha_max_endstop")
#define beta_max_endstop_checksum        CHECKSUM("beta_max_endstop")
#define gamma_max_endstop_checksum       CHECKSUM("gamma_max_endstop")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

#define abs(a) ((a<0) ? -a : a)
#define sign(a) ((a<0) ? -1 : 1)

#define AVG_LEN 128

Joystick::Joystick() {}

Joystick::~Joystick() {}

void Joystick::on_module_loaded()
{
    if( !THEKERNEL->config->value( joystick_checksum, enable_checksum )->by_default(false)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

    //load configuration for this module
    this->on_config_reload(this);

    //register for some events with the kernel
    this->register_for_event(ON_GCODE_RECEIVED); //ask to be notified when gcode is received, it might be for us
    THEKERNEL->slow_ticker->attach(100, this, &Joystick::update_tick); //not sure what this was doing
    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){acceleration_tick(); }); //also not sure what this was doing

    /*
    TODO:
        add automatic detection of steady state, add config value which when set will run the procedure, then set it back to false
        add safety checks, no moving faster than absolute max, check if queue empty is valid check for busy with commands
        customizable speed per axis
        customizable axis to control
        selectable speed settings, fast/slow (handle a custom M code for this? then switch module can make the selection via gcode)

        possible future change: make one module for reading joystick values and mapping them between -1 and 1
        then have a "Jog" module which manages the motor movements. the user can choose which applicable module to obtain the -1 to 1 readings
    */

    //read position repeatedly to get an average for "stationary" joystick values, maybe need to register a slow timer with the kernel for this
    /*
    int read_sum[2] = {0, 0};

    for(int c = 0; c < 2; c++){
        int* pos;
        for(int i = 0; i < AVG_LEN; i++){
            pos = read_pos();
            read_sum[c] += pos[c];
        }
        zero_offset[c] = read_sum[c] / AVG_LEN;
    }
    */
}

void Joystick::on_config_reload(void *argument)
{
    //read config file values for this module
    this->max_speed = THEKERNEL->config->value(default_seek_rate_checksum )->by_default(10.0f )->as_number();
    this->max_speed = THEKERNEL->config->value(joystick_checksum, max_speed_checksum )->by_default(this->max_speed )->as_number();
    this->dead_zone = THEKERNEL->config->value(joystick_checksum, dead_zone_checksum )->by_default(0.002f )->as_number();
    this->zero_offset[0] = THEKERNEL->config->value(joystick_checksum, zero_offset_horizontal_checksum )->by_default(2047 )->as_number();
    this->zero_offset[1] = THEKERNEL->config->value(joystick_checksum, zero_offset_vertical_checksum )->by_default(2047 )->as_number();
    this->nonlinearity = THEKERNEL->config->value(joystick_checksum, nonlinearity_checksum )->by_default(1.0f )->as_number();

    // pins for ADC readings
    this->axis_pin[0].from_string(THEKERNEL->config->value(joystick_checksum, horizontal_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&axis_pin[0]);

    this->axis_pin[1].from_string(THEKERNEL->config->value(joystick_checksum, vertical_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&axis_pin[1]);

    //pins for endstops (currently having to check manually before issuing stepper move function)
    this->endstop_pin[0].from_string( THEKERNEL->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->endstop_pin[1].from_string( THEKERNEL->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->endstop_pin[2].from_string( THEKERNEL->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->endstop_pin[3].from_string( THEKERNEL->config->value(alpha_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->endstop_pin[4].from_string( THEKERNEL->config->value(beta_max_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->endstop_pin[5].from_string( THEKERNEL->config->value(gamma_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
}

//read joystick position, takes multiple readings and removes outliers (> 1 stdev from mean)
int* Joystick::read_pos()
{
    static int m[2];

    for(int c = 0; c < 2; c++){
        int last_raw = THEKERNEL->adc->read(&axis_pin[c]);
        if (queue[c].size() >= queue[c].capacity()) {
            uint16_t l;
            queue[c].pop_front(l);
        }
        uint16_t r = last_raw;
        queue[c].push_back(r);

        //get the mean squared value
        float std_sum = 0;
        for (int i=0; i<queue[c].size(); i++) {
            std_sum += pow(*queue[c].get_ref(i), 2);
        }
        std_sum /= queue[c].size(); //E[X^2]

        //get the mean value
        float avg_sum = 0;
        for (int i=0; i<queue[c].size(); i++) {
            avg_sum += *queue[c].get_ref(i);
        }
        avg_sum /= queue[c].size(); //E[X]

        //compute standard deviation
        float std = sqrt(std_sum - pow(avg_sum, 2)); //sqrt(E[X^2] - E[X]^2)

        //compute average without outliers
        float avg = 0;
        int avg_num = 0;
        for (int i=0; i<queue[c].size(); i++) {
            if((*queue[c].get_ref(i) - avg_sum) < std){
                avg += *queue[c].get_ref(i);
                avg_num++;
            }
        }
        avg /= avg_num;
        m[c] = (int) floorf(avg);
    }
    return m;
}

//get normalized joystick position from -1 to 1
float* Joystick::get_normalized(int* pos)
{
    int pos_zero[2];
    static float norm[2];

    for(int i = 0; i < 2; i++){
        //first convert 0 to 4095 to +/- centered on zero
        pos_zero[i] = (pos[i] - zero_offset[i]);

        //then scale the output to +/- 1 boundary
        norm[i] = pos_zero[i] / (float) min(4095 - zero_offset[i], zero_offset[i]);

        //constrain to within +/- 1
        if(norm[i] < -1 || norm[i] > 1){
            norm[i] = sign(norm[i]);
        }
        
    }

    return norm;
}

//map a position from -1 to 1 into a speed from -max to max
float Joystick::get_speed(float pos){
    float speed;

    //apply non-linear scaling to position
    speed = (sign(pos) * pow(abs(pos), nonlinearity)) * max_speed;

    //zero out in dead zone
    if(abs(pos) <= dead_zone){
        speed = 0;
    }

    return speed;
}

void Joystick::on_gcode_received(void *argument)
{
    //testing code here
    //print out parameters
    //int* pos = read_pos();
    //THEKERNEL->streams->printf("%+0.3f, %+0.3f          Max: %f,  Dead: %f,  Zero: %d, %d, Pw: %f,  ADCX: %d,  ADCY: %d\n", this->position[0], this->position[1], max_speed, dead_zone, zero_offset[0], zero_offset[1], nonlinearity, pos[0], pos[1]);
}

uint32_t Joystick::update_tick(uint32_t dummy)
{
    //update the current joystick position
    this->position = get_normalized(read_pos());

    //don't bother doing anything if the robot is busy with commands
    if(!THEKERNEL->conveyor->is_queue_empty()) return 0;

    //update stepper's target speed
    for(int c = 0; c < 2; c++){
        this->target_feedrate[c] = get_speed(this->position[c]) * STEPS_PER_MM(c);
        
        //stop here if any endstop safety checks fail (only checking in direction of travel)
        if(this->endstop_pin[c + ((target_feedrate[c]<0)? 0 : 3)].get()) continue;
        
        //enable the stepper if the target speed is non-zero
        if(!this->enabled[c] && target_feedrate[c] != 0){
            this->enabled[c] = true;
            THEKERNEL->stepper->turn_enable_pins_on();
        }
    }

    //if jogging, update the robot's last milestone (i.e. position) to the steppers' current positions
    THEKERNEL->robot->reset_position_from_current_actuator_position();

    return 0;
}

// Called periodically to change the speed to match acceleration
void Joystick::acceleration_tick(void)
{
    //update enabled steppers' speeds
    for(int c = 0; c < 2; c++){
        if(enabled[c]) accelerate(c);
    }

    return;
}

//acceleration calculation, just need to set this->current_feedrate
void Joystick::accelerate(int c)
{   
    float current_rate = (direction[c])? STEPPER[c]->get_steps_per_second() : -1*(STEPPER[c]->get_steps_per_second());
    if(!STEPPER[c]->is_moving()) current_rate = 0;

    float target_rate = this->target_feedrate[c];
    float old_rate = current_rate;

    // Z may have a different acceleration to X and Y
    float acc = (c==Z_AXIS) ? THEKERNEL->planner->get_z_acceleration() : THEKERNEL->planner->get_acceleration();
    float rate_change = (acc / THEKERNEL->acceleration_ticks_per_second) * STEPS_PER_MM(c);

    //update the current rate using the acceleration
    if( current_rate <= target_rate ) {
        current_rate = min( target_rate, current_rate + rate_change );
    } else {
        current_rate = max( target_rate, current_rate - rate_change );
    }

    //check if current rate is zero, stop motors if so
    if(current_rate == 0){
        STEPPER[c]->move(0, 0);
        enabled[c] = false;

    } else {
        //check if switched sign from last speed to new speed, or last speed was 0
        if(sign(old_rate) != sign(current_rate) || old_rate == 0){
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
