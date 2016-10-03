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

    /*
    TODO:
        add automatic detection of steady state, add config value which when set will run the procedure, then set it back to false
        add safety checks, no moving faster than absolute max, check if queue empty is valid check for busy with commands
        customizable speed per axis
        customizable axis to control
        selectable speed settings, fast/slow (handle a custom M code for this? then switch module can make the selection via gcode)

        possible future change: make one module for reading joystick values and mapping them between -1 and 1
        Make Joystick module have multiple instances like the extruder module, one for each possible analog axis
        Make Joystick module share its position using SharedData? then any interested module can use the ON_SET_PUBLIC_DATA event to get the reading
        OR Joystick module is able to respond to a SharedData request, any interested module asks for the data

        have a "Jog" module which manages the motor movements given an input from -1 to 1. Can be configured to use any module,
        just needs to be a module that supports the public data getting/setting
    */

    //read position repeatedly to get an average for "stationary" joystick values, maybe need to register a slow timer with the kernel for this
    //probably best to have a "startup time" where the module is inactive, and is obtaining readings to average, only if this "startup time" is enabled in config
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
    this->zero_offset[0] = THEKERNEL->config->value(joystick_checksum, zero_offset_horizontal_checksum )->by_default(2047 )->as_number();
    this->zero_offset[1] = THEKERNEL->config->value(joystick_checksum, zero_offset_vertical_checksum )->by_default(2047 )->as_number();

    // pins for ADC readings
    this->axis_pin[0].from_string(THEKERNEL->config->value(joystick_checksum, horizontal_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&axis_pin[0]);

    this->axis_pin[1].from_string(THEKERNEL->config->value(joystick_checksum, vertical_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&axis_pin[1]);
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
        if(abs(norm[i]) > 1){
            norm[i] = sign(norm[i]);
        }
        
    }

    return norm;
}


uint32_t Joystick::update_tick(uint32_t dummy)
{
    //update the current joystick position
    this->position = get_normalized(read_pos());

    return 0;
}
