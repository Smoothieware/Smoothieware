
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
#define segment_frequency_checksum          CHECKSUM("step_frequency")

#define axis0_data_source_checksum          CHECKSUM("data_source_alpha")
#define axis1_data_source_checksum          CHECKSUM("data_source_beta")
#define axis2_data_source_checksum          CHECKSUM("data_source_gamma")
#define axis3_data_source_checksum          CHECKSUM("data_source_delta")
#define axis4_data_source_checksum          CHECKSUM("data_source_epsilon")
#define axis5_data_source_checksum          CHECKSUM("data_source_zeta")

#define m_code_set_checksum                 CHECKSUM("m_code_set")
#define m_code_toggle_checksum              CHECKSUM("m_code_toggle")
#define jog_axes_checksum                   CHECKSUM("jog_axes")

#define abs(a) ((a<0.0f) ? -a : a)
#define sign(a) ((a<0.0f) ? -1 : 1)

Jogger::Jogger() {
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        this->position[c] = 0.0f;
        this->target_speed[c] = 0.0f;
    }
}

//TODO: find examples of other modules, determine if necessary
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

    //register for the main loop in the kernel
    this->register_for_event(ON_MAIN_LOOP);

    //ask the kernel to run "update_tick" at "refresh_rate" Hz
    THEKERNEL->slow_ticker->attach(this->refresh_rate, this, &Jogger::update_tick);

}

// respond to possible M-codes for jog axis change
void Jogger::on_gcode_received(void *argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m) {
        //TODO: add code to respond to these M-codes
        if (gcode->m == this->m_code_set) {
            //update the current machine jog axes to the given string
            //note the +1 is because the gcode command will have a leading space which needs to be ignored
            update_Axes(gcode->get_command()+1);
        }
        else if (gcode->m == this->m_code_toggle) {
            //increment the index of the current machine axis settings
            this->axis_index++;
            if (this->axis_index == this->jog_axes.size()) {
                this->axis_index = 0;
            }

            //update the current machine axis settings
            update_Axes(this->jog_axes[this->axis_index]);

            //debug only
            THEKERNEL->streams->printf("Jog Axis Preset %d of %d\n", this->axis_index+1, this->jog_axes.size());
        }
        else if (gcode->m == 776) {
            //debug only, print the Gcode that is being run in the jog loop
            std::string g = get_Gcode();
            THEKERNEL->streams->printf(">>> %s\n", g.c_str());

            //print each axis letter
            for (unsigned int i = 0; i < NUM_JOG_AXES; i++) {
                THEKERNEL->streams->printf(">>> %c\n", this->axis_letter[i]);
            }
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
    this->segment_frequency = THEKERNEL->config->value(jogger_checksum, segment_frequency_checksum)->by_default(this->segment_frequency)->as_number();
    this->m_code_set = THEKERNEL->config->value(jogger_checksum, m_code_set_checksum)->by_default(this->m_code_set)->as_number();
    this->m_code_toggle = THEKERNEL->config->value(jogger_checksum, m_code_toggle_checksum)->by_default(this->m_code_toggle)->as_number();
    
    //load the names of the joystick modules where each axis will get its data
    uint16_t axisN_data_source_checksum[] = { axis0_data_source_checksum, axis1_data_source_checksum, axis2_data_source_checksum, axis3_data_source_checksum, axis4_data_source_checksum, axis5_data_source_checksum };
    for (int i = 0; i < NUM_JOG_AXES; i++) {
        this->axis_data_source[i] = get_checksum(THEKERNEL->config->value(jogger_checksum, axisN_data_source_checksum[i])->by_default("")->as_string());
    }

    //load the machine axis letters that the joystick will control
    std::string letterlist = THEKERNEL->config->value(jogger_checksum, jog_axes_checksum)->by_default("XYZABC")->as_string();

    //parse the letter list into substrings
    this->jog_axes = split(letterlist.c_str(), ',');

    //update the current machine axis settings
    update_Axes(this->jog_axes[this->axis_index]);
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

//runs on a timer to update the jog speeds (default 100 Hz update rate)
uint32_t Jogger::update_tick(uint32_t dummy)
{
    //set a flag indicating we need to update the joystick reading when in the main loop
    this->do_reading = true;

    return 0;
}

//runs whenever the smoothie is in the main loop, safe to send Gcode here
void Jogger::on_main_loop(void *argument)
{   
    if (this->do_reading) {
        //flag was set, means it's time to update the joystick reading
        update_Joystick();
        this->do_reading = false;
    }

    //update the jogging functionality
    update_Jogging();

}

//perform a public data request from the joystick, also determine from the result if the joystick is active
void Jogger::update_Joystick(void)
{
    bool allzero = true; //will determine whether the joystick is at rest or not

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

    //set the joystick's current activity (active if not all axes are zero)
    this->is_active = !allzero;
}

//perform tasks pertinent to axis movement and enabling/disabling the jogger
void Jogger::update_Jogging(void)
{
    //THREE POSSIBLE STATES TO CHECK:
    //1: joystick active and jogging -> keep jogging
    //2: joystick active but not jogging -> start jogging
    //3: joystick inactive but still jogging -> stop jogging when the machine has stopped
    //(4): joystick inactive and not jogging -> do nothing

    //check if the joystick is now active
    if (this->is_active) {
        //check if we're currently jogging
        if (this->is_jogging) {
            //joystick both active and jogging, keep jogging
            //add moves to the conveyor, only if it is not already full with moves to be done
            if (!THECONVEYOR->is_queue_full()) {
                //get the Gcode needed to make a small move in the right direction, and submit it
                std::string g = get_Gcode();
                Gcode gc(g, &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
            }
        }
        else {
            //joystick active but not jogging, may need to start jogging
            //only activate the module if the conveyor is idle (i.e. not half-way through a job/print)
            if (THECONVEYOR->is_idle()) {
                this->is_jogging = true;

                //save the current robot state (abs/rel mode, seek rate)
                THEROBOT->push_state();

                //change the mode to relative with G91
                Gcode gcrel("G91", &(StreamOutput::NullStream));
                THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcrel);
            }
        }
    }
    //else, check if the joystick is still jogging
    else if (this->is_jogging) {
        //check if the robot is done moving from previous jogging
        if (THECONVEYOR->is_idle()) {
            this->is_jogging = false;

            //restore the robot's state before jogging (abs/rel mode, seek rate)
            THEROBOT->pop_state();
        }
    }
}

//create a new G-code to move a small distance in the direction given by the joystick, at the speed defined by the joystick position
//TODO: check if need to pass string by reference
std::string Jogger::get_Gcode(void)
{
    //get the magnitude of the speed (sqrt of sum of axis speeds squared)
    float spd = 0.0f;
    for (int c = 0; c < NUM_JOG_AXES; c++) {
        spd += (this->target_speed[c] * this->target_speed[c]);
    }
    spd = sqrtf(spd);

    //use segment frequency (f) to calculate step scale factor (ssf (mm/segment) = speed (mm/s) / f (segments/s))
    float ssf = spd / 60.0f / this->segment_frequency;

    //build the Gcode up
    char command[64];
    int n = 0;

    //start with a G1
    n += snprintf(command + n, sizeof(command) - n, "G1");

    //for each possible jog axis letter
    for (int i = 0; i < NUM_JOG_AXES; i++) {
        //skip the letter if it isn't a valid machine axis letter
        if (this->axis_letter[i] != 'X' &&
            this->axis_letter[i] != 'Y' &&
            this->axis_letter[i] != 'Z' &&
            this->axis_letter[i] != 'A' &&
            this->axis_letter[i] != 'B' &&
            this->axis_letter[i] != 'C') {
            continue;
        }

        //append the machine axis letter and its required distance
        n += snprintf(command + n, sizeof(command) - n, " %c%1.2f", this->axis_letter[i], this->position[i] * ssf);
    }

    //append a feedrate command
    n += snprintf(command + n, sizeof(command) - n, " F%1.1f", spd);

    //return the string
    std::string g(command, n);
    return g;
}

//update the current machine axis settings
//TODO: check if need to pass string by reference
void Jogger::update_Axes(std::string axisstr)
{
    //loop through all possible jog axes
    for (unsigned int i = 0; i < NUM_JOG_AXES; i++) {
        if (i < axisstr.length()) {
            //get the letter for this axis in the list
            this->axis_letter[i] = toupper(axisstr[i]);
        }
        else { //no character specified, use signal char = '-' to indicate no axis to control
            this->axis_letter[i] = '-';
        }
    }
}