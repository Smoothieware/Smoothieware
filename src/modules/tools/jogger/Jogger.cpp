
#include "Jogger.h"

#include <math.h>
#include "Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"

#include "JoystickPublicAccess.h"
#include "PublicData.h"
#include "utils.h"

#include "StreamOutputPool.h" //just for debugging


#define jogger_checksum                     CHECKSUM("jogger")
#define refresh_interval_checksum           CHECKSUM("refresh_interval")

#define axis0_data_source_checksum          CHECKSUM("data_source_alpha")
#define axis1_data_source_checksum          CHECKSUM("data_source_beta")
#define axis2_data_source_checksum          CHECKSUM("data_source_gamma")

Jogger::Jogger() {}

void Jogger::on_module_loaded()
{

    //load configuration for this module
    this->on_config_reload(this);

    //register for events with the kernel
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);

    //ask the kernel to run "update_tick" every "refresh_interval" milliseconds
    THEKERNEL->slow_ticker->attach(1000 / this->refresh_interval, this, &Jogger::update_tick);

}

//debug only
void Jogger::on_gcode_received(void *argument)
{
    //testing code here
    //print out parameters
    int pos = -10;
    float posf = -10;

    //test a public data read
    struct PAD_joystick s;
    if (PublicData::get_value(joystick_checksum, this->axis_data_source[0], &s)) {
        pos = s.raw;
        posf = s.position;
    }
    else {
        THEKERNEL->streams->printf("Error reading target %d\n", this->axis_data_source[0]);
    }

    THEKERNEL->streams->printf("%+0.2f  (%d)    Int: %d\n", posf, pos, refresh_interval);
}

//read config file values for this module
void Jogger::on_config_reload(void *argument)
{

    this->refresh_interval = THEKERNEL->config->value(jogger_checksum, refresh_interval_checksum)->by_default(this->refresh_interval)->as_number();
    

    uint16_t axisN_data_source_checksum[] = { axis0_data_source_checksum, axis1_data_source_checksum, axis2_data_source_checksum };
    for (int i = 0; i < NUM_JOG_AXES; i++) {
        this->axis_data_source[i] = get_checksum(THEKERNEL->config->value(jogger_checksum, axisN_data_source_checksum[i])->by_default("")->as_string());
    }
    
}

//runs on a timer to update the jog speeds
uint32_t Jogger::update_tick(uint32_t dummy)
{

    return 0;
}
