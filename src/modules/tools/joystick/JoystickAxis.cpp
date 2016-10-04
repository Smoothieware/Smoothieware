
#include "JoystickAxis.h"
#include "JoystickPublicAccess.h"

#include <math.h>
#include "Kernel.h"
//#include "Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
//#include "Robot.h"
#include "SlowTicker.h"
#include "PublicDataRequest.h"


#define joystick_checksum                   CHECKSUM("joystick")
#define pin_checksum                        CHECKSUM("pin")
#define zero_offset_checksum                CHECKSUM("zero_offset")
#define endpoint_checksum                   CHECKSUM("endpoint")
#define auto_zero_checksum                  CHECKSUM("auto_zero")
#define startup_time_checksum               CHECKSUM("startup_time")
#define refresh_interval_checksum           CHECKSUM("refresh_interval")
#define start_value_checksum                CHECKSUM("start_value")


#define abs(a) ((a<0) ? -a : a)
#define sign(a) ((a<0) ? -1 : 1)


JoystickAxis::JoystickAxis() {}

JoystickAxis::JoystickAxis(uint16_t name)
{
    this->name_checksum = name;
}

void JoystickAxis::on_module_loaded()
{

    //load configuration for this module
    this->on_config_reload(this);

    //register for events with the kernel
    this->register_for_event(ON_GET_PUBLIC_DATA);

    //ask the kernel to run "update_tick" every "refresh_interval" milliseconds
    THEKERNEL->slow_ticker->attach(this->refresh_interval, this, &JoystickAxis::update_tick);

}

//read config file values for this module
void JoystickAxis::on_config_reload(void *argument)
{

    //pin for ADC readings
    this->axis_pin.from_string(THEKERNEL->config->value(joystick_checksum, this->name_checksum, pin_checksum)->by_default("nc")->as_string())->as_input();
    THEKERNEL->adc->enable_pin(&axis_pin);

    //other config options
    this->zero_offset = THEKERNEL->config->value(joystick_checksum, this->name_checksum, zero_offset_checksum)->by_default(this->zero_offset)->as_number();
    this->endpoint = THEKERNEL->config->value(joystick_checksum, this->name_checksum, endpoint_checksum)->by_default(THEKERNEL->adc->get_max_value())->as_number();
    this->auto_zero = THEKERNEL->config->value(joystick_checksum, this->name_checksum, auto_zero_checksum)->by_default(false)->as_bool();
    this->startup_time = THEKERNEL->config->value(joystick_checksum, this->name_checksum, startup_time_checksum)->by_default(this->startup_time)->as_number();
    this->refresh_interval = THEKERNEL->config->value(joystick_checksum, this->name_checksum, refresh_interval_checksum)->by_default(this->refresh_interval)->as_number();
    this->position = THEKERNEL->config->value(joystick_checksum, this->name_checksum, start_value_checksum)->by_default(this->position)->as_number();
    
}

void JoystickAxis::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    //check if the request is for a joystick module, return if not
    if (!pdr->starts_with(joystick_checksum)) return;

    //check if the request is for this particular joystick module, return if not
    if (!pdr->second_element_is(this->name_checksum)) return;

    // caller has provided the location to write the state to
    struct PAD_joystick* pad = static_cast<struct PAD_joystick *>(pdr->get_data_ptr());
    pad->name_checksum = this->name_checksum;
    pad->position = this->position;
    pdr->set_taken();
}

//read joystick position, uses past readings to removes outliers (> 1 stdev from mean)
int JoystickAxis::read_pos()
{

    //get the raw ADC value
    unsigned int last_raw = THEKERNEL->adc->read(&axis_pin);
    if (queue.size() >= queue.capacity()) {
        uint16_t l;
        queue.pop_front(l);
    }
    uint16_t r = last_raw;
    queue.push_back(r);

    //get the mean squared value
    float std_sum = 0;
    for (int i=0; i<queue.size(); i++) {
        std_sum += pow(*queue.get_ref(i), 2);
    }
    std_sum /= queue.size(); //E[X^2]

    //get the mean value
    float avg_sum = 0;
    for (int i=0; i<queue.size(); i++) {
        avg_sum += *queue.get_ref(i);
    }
    avg_sum /= queue.size(); //E[X]

    //compute standard deviation
    float std = sqrt(std_sum - pow(avg_sum, 2)); //sqrt(E[X^2] - E[X]^2)

    //compute average without outliers
    float avg = 0;
    int avg_num = 0;
    for (int i=0; i<queue.size(); i++) {
        if((*queue.get_ref(i) - avg_sum) < std){
            avg += *queue.get_ref(i);
            avg_num++;
        }
    }
    avg /= avg_num;

    return (int) round(avg);
}

//get normalized joystick position from -1 to 1
float JoystickAxis::get_normalized(int pos)
{
    int pos_zero;
    float norm;

    //first convert 0 to Adc.get_max_value() to +/- centered on zero
    pos_zero = (pos - zero_offset);

    //then scale the output to +/- 1 boundary
    norm = pos_zero / abs(endpoint - zero_offset);

    //constrain to within +/- 1
    if(abs(norm) > 1) {
        norm = sign(norm);
    }

    return norm;
}


uint32_t JoystickAxis::update_tick(uint32_t dummy)
{

    //if still in the "startup" period and auto-zero is enabled
    if (this->in_startup && this->auto_zero) {
        //add the current ADC measurement to the running sum
        this->startup_sum += read_pos();

        //increment the number of intervals done so far
        this->startup_intervals++;

        //check if next step would be out of the startup time
        if ((this->startup_intervals+1)*this->refresh_interval > this->startup_time) {
            //finalize the zero-offset as the average of the readings during the startup time
            this->zero_offset = this->startup_sum / this->startup_intervals;

            //exit startup mode
            this->in_startup = false;
            return 0;
        }
    }
    else { //not in startup mode or auto-zero was disabled
        //update the current joystick position
        this->position = get_normalized(read_pos());
    }

    return 0;
}
