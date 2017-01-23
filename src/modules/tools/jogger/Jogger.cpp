
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
#include "string.h"

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

#define abs(a) ((a<0.0f) ? -a : a)
#define sign(a) ((a<0.0f) ? -1 : 1)

Jogger::Jogger() {}

//deconstructor: THEKERNEL->unregister_for_event(ON_GCODE_RECEIVED, this); //unregister for on_main_loop too

void Jogger::on_module_loaded()
{
    if (!THEKERNEL->config->value(jogger_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as not needed free up resource
        delete this;
        return;
    }

    //load configuration for this module
    this->on_config_reload(this);

    //register for GCode events with the kernel
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);

}

//add config for which M-code(s) to respond to
//add code to respond to M-codes for plane change
void Jogger::on_gcode_received(void *argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m) {
        if (gcode->m == 777) {
            //print out parameters
            THEKERNEL->streams->printf("%+0.2f, %+0.2f    Max: %0.1f, Dead: %f, Nl: %f, SF: %f, Rate: %d\n", this->position[0], this->position[1], max_speed, dead_zone, nonlinearity, step_scale_factor, refresh_rate);
        }
    }
}

//read config file values for this module
void Jogger::on_config_reload(void *argument)
{
    //load the basic configuration settings
    this->max_speed = THEKERNEL->config->value(default_seek_rate_checksum)->by_default(this->max_speed)->as_number();
    this->max_speed = THEKERNEL->config->value(jogger_checksum, max_speed_checksum)->by_default(this->max_speed)->as_number();
    this->dead_zone = THEKERNEL->config->value(jogger_checksum, dead_zone_checksum)->by_default(this->dead_zone)->as_number();
    this->nonlinearity = THEKERNEL->config->value(jogger_checksum, nonlinearity_checksum)->by_default(this->nonlinearity)->as_number();
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

    //zero out in dead zone
    if (abs(pos) < this->dead_zone) {
        speed = 0.0f;
    }
    else {
        //apply non-linear scaling to position
        speed = (sign(pos) * pow(abs(pos), this->nonlinearity)) * this->max_speed;
    }

    return speed;
}

//runs on a timer to update the jog speeds
void Jogger::on_main_loop(void *argument)
{
    bool allzero = true; //determines whether the joystick is at rest or not

    //read the joysticks for each axis and map their values to a target speed
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        struct PAD_joystick s;
        if (PublicData::get_value(joystick_checksum, this->axis_data_source[c], &s)) {
            this->position[c] = s.position;
        }
        else {
            this->position[c] = 0.0f;
            //position should be 0 if the module doesn't respond, or the module name is unspecified/wrong
            //no movement on that axis if position is 0 = safe
        }

        //determine if this axis' speed is non-zero (roughly)
        if (abs(this->position[c]) >= this->dead_zone) {
            allzero = false;
        }
        
        //map the joystick position to a speed
        this->target_speed[c] = get_speed(this->position[c]);
    }

    //check if the joystick is inactive (i.e. all axes of the joystick are roughly zero)
    if (allzero) {
        //the joystick is inactive but if the jogger is active
        if (this->is_active) {
            //then check if the robot is done moving from previous jogging
            if (THECONVEYOR->is_idle()) {
                //and disable the jogger if so
                this->is_active = false;
                THEKERNEL->streams->printf(">>> DISABLED\n");
            }
        }
        //break from the function, no further actions needed
        return;
    }

    //the joystick is active, check if the module was previously inactive
    if (!this->is_active) {
        //if so, only activate the module if the conveyor is idle
        if (THECONVEYOR->is_idle()) {
            this->is_active = true;
            THEKERNEL->streams->printf(">>> ENABLED\n");
        }
        else {
            //otherwise, the conveyor is not ready to have moves added, so return
            return;
        }
    }
    
    //now that the joystick is deemed active, and the module is safe to be active
    //add moves to the conveyor, only if it is not already full with moves to be done
    if (!THECONVEYOR->is_queue_full()) {
        //check if the robot is in absolute mode, change to relative if so by sending Gcode
        Gcode gcrel("G91", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcrel);
        
        //create a new G-code to move a small distance in the direction given by the joystick, at the speed defined by the joystick position
        //TODO: build this command using variable axis letters
        //e.g. this->position[0] comes from data_source_alpha, which maps to axis "X"
        //the axis mapping should be changeable through M-code or maybe a plane select G-code (e.g. 17/18/19)
        //TODO: calc total speed from whatever number of axes exist in loop
        char command[32];
        int n = snprintf(command, sizeof(command), "G1 X%1.2f Y%1.2f F%1.1f", this->position[0] * this->step_scale_factor, this->position[1] * this->step_scale_factor, sqrtf(powf(this->target_speed[0], 2.0f) + powf(this->target_speed[1], 2.0f)));
        std::string g(command, n);
        Gcode gc(g, &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

        //debug use: echo the command
        //THEKERNEL->streams->printf(">>> %s\n", command);

        //manually resend a G-code if required to put the machine back into absolute
        Gcode gcabs("G90", &(StreamOutput::NullStream));
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcabs);
    }

    return;
}