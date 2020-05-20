#include "TemperatureSwitch.h"
#include "Kernel.h"
#include "checksumm.h"
#include "utils.h"
#include "Test_kernel.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "TemperatureControlPublicAccess.h"
#include "SwitchPublicAccess.h"
#include "Gcode.h"

#include <stdio.h>
#include <memory>

#include "easyunit/test.h"

// this declares any global variables the test needs
DECLARE(TemperatureSwitch)
  TemperatureSwitch *ts;
END_DECLARE

// called before each test
SETUP(TemperatureSwitch)
{
    // create the module we want to test
    ts = new TemperatureSwitch();
    //printf("...Setup TemperatureSwitch\n");
}

// called after each test
TEARDOWN(TemperatureSwitch)
{
    // delete the module
    delete ts;

    // have kernel reset to a clean state
    test_kernel_teardown();

    //printf("...Teardown TemperatureSwitch\n");
}

// define various configs here, these are in the same formate they would appeat in the config file (comments removed for clarity)
const static char edge_low_config[]= "\
temperatureswitch.psu_off.enable true \n\
temperatureswitch.psu_off.designator T \n\
temperatureswitch.psu_off.switch fan \n\
temperatureswitch.psu_off.threshold_temp 50.0 \n\
temperatureswitch.psu_off.heatup_poll 5 \n\
temperatureswitch.psu_off.cooldown_poll 5 \n\
temperatureswitch.psu_off.arm_mcode 1100 \n\
temperatureswitch.psu_off.trigger falling \n\
temperatureswitch.psu_off.inverted false \n\
";

const static char level_config[]= "\
temperatureswitch.psu_off.enable true \n\
temperatureswitch.psu_off.designator T \n\
temperatureswitch.psu_off.switch fan \n\
temperatureswitch.psu_off.threshold_temp 50.0 \n\
temperatureswitch.psu_off.heatup_poll 5 \n\
temperatureswitch.psu_off.cooldown_poll 5 \n\
";

// handle mock call to temperature control
// simulates one temperature control with T designator and id of 123
static void on_get_public_data_tc1(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(temperature_control_checksum)) return;
    if(!pdr->second_element_is(poll_controls_checksum)) return;

    std::vector<struct pad_temperature> *v= static_cast<std::vector<pad_temperature>*>(pdr->get_data_ptr());

    struct pad_temperature t;
    // setup data
    t.designator= "T";
    t.id= 123;
    v->push_back(t);
    pdr->set_taken();
}

// Handle temperature request and switch status request
static bool switch_state;
static bool switch_get_hit= false;
static int hitcnt= 0;
static float return_current_temp= 0;
static void on_get_public_data_tc2(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(pdr->starts_with(temperature_control_checksum)) {
        // return a current temperature of return_current_temp when requested
        if(!pdr->second_element_is(current_temperature_checksum)) return;
        if(!pdr->third_element_is(123)) return;

        hitcnt++;

        struct pad_temperature *t= static_cast<pad_temperature*>(pdr->get_data_ptr());
        t->current_temperature = return_current_temp;
        t->target_temperature = 0;
        t->pwm = 0;
        t->designator= "T";
        t->id= 123;
        pdr->set_taken();

    } else if(pdr->starts_with(switch_checksum)){
        // return status of the fan switch
        if(!pdr->second_element_is(get_checksum("fan"))) return;

        switch_get_hit= true;

        static struct pad_switch *pad= static_cast<pad_switch*>(pdr->get_data_ptr());;
        pad->name = get_checksum("fan");
        pad->state = switch_state;
        pad->value = 0;
        pdr->set_taken();
    }
}

// simulates the switch being turned on or off so we can test the state
static bool switch_set_hit= false;
static void on_set_public_data_switch(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;
    if(!pdr->second_element_is(get_checksum("fan"))) return;
    if(!pdr->third_element_is(state_checksum)) return;

    switch_set_hit= true;

    bool t = *static_cast<bool *>(pdr->get_data_ptr());
    switch_state = t;
    pdr->set_taken();
}

// stimulates the module with on_second ticks in which it asks for a temperature which we simulate above
static bool set_temp(TemperatureSwitch *nts, float t)
{
    // trap public data request to TemperatureControl and return a temperature
    hitcnt= 0;
    return_current_temp= t;
    test_kernel_trap_event(ON_GET_PUBLIC_DATA, on_get_public_data_tc2);

    // tick 5 times for 5 seconds
    for (int i = 0; i < 5; ++i) {
        nts->on_second_tick(nullptr);
    }

    // make sure the temp was read at least once
    return (hitcnt > 0);
}

TESTF(TemperatureSwitch,level_low_high)
{
    // load config with required settings for this test
    test_kernel_setup_config(level_config, &level_config[sizeof(level_config)]);

    // trap public data request to TemperatureControl and return a mock tempcontrol
    test_kernel_trap_event(ON_GET_PUBLIC_DATA, on_get_public_data_tc1);

    // make module load the config
    uint16_t cs= get_checksum("psu_off");
    std::unique_ptr<TemperatureSwitch> nts(ts->load_config(cs));

    if(nts.get() == nullptr) {
        FAIL_M("load config failed");
    }

    // stop handling this event
    test_kernel_untrap_event(ON_GET_PUBLIC_DATA);

    // test it registered the event
    ASSERT_TRUE(THEKERNEL->kernel_has_event(ON_GCODE_RECEIVED, nts.get()));

    // set the first temperature
    ASSERT_TRUE(set_temp(nts.get(), 25));

    // capture any call to the switch to turn it on or off
    switch_state= false;
    switch_set_hit= false;
    switch_get_hit= false;
    test_kernel_trap_event(ON_SET_PUBLIC_DATA, on_set_public_data_switch);

    // increase temp low -> high
    ASSERT_TRUE(set_temp(nts.get(), 60));

    ASSERT_TRUE(switch_get_hit);
    // make sure switch was set
    ASSERT_TRUE(switch_set_hit);

    // and make sure it was turned on
    ASSERT_TRUE(switch_state);

    // now make sure it turns off when temp drops
    ASSERT_TRUE(set_temp(nts.get(), 30));

    // and make sure it was turned off
    ASSERT_TRUE(!switch_state);
}

TESTF(TemperatureSwitch,edge_high_low)
{
    // load config with required settings for this test
    test_kernel_setup_config(edge_low_config, &edge_low_config[sizeof(edge_low_config)]);

    // trap public data request to TemperatureControl and return a mock tempcontrol
    test_kernel_trap_event(ON_GET_PUBLIC_DATA, on_get_public_data_tc1);

    // make module load the config
    uint16_t cs= get_checksum("psu_off");
    std::unique_ptr<TemperatureSwitch> nts(ts->load_config(cs));

    if(nts.get() == nullptr) {
        FAIL_M("load config failed");
    }

    // stop handling this event
    test_kernel_untrap_event(ON_GET_PUBLIC_DATA);

    // test it registered the event
    ASSERT_TRUE(THEKERNEL->kernel_has_event(ON_GCODE_RECEIVED, nts.get()));
    ASSERT_TRUE(!nts->is_armed());

    // set initial temp low
    ASSERT_TRUE(set_temp(nts.get(), 25));

    // capture any call to the switch to turn it on or off
    switch_state= true;
    test_kernel_trap_event(ON_SET_PUBLIC_DATA, on_set_public_data_switch);

    // increase temp low -> high
    ASSERT_TRUE(set_temp(nts.get(), 60));

    // make sure it was not turned off
    ASSERT_TRUE(switch_state);

    // drop temp
    ASSERT_TRUE(set_temp(nts.get(), 30));

    // and make sure it was still on
    ASSERT_TRUE(switch_state);

    // now arm it
    Gcode gc("M1100 S1", (StreamOutput *)THEKERNEL->serial, false);
    nts->on_gcode_received(&gc);

    ASSERT_TRUE(nts->is_armed());

    // increase temp low -> high
    ASSERT_TRUE(set_temp(nts.get(), 60));

    // make sure it was not turned off
    ASSERT_TRUE(switch_state);

    // drop temp
    ASSERT_TRUE(set_temp(nts.get(), 30));

    // and make sure it was turned off
    ASSERT_TRUE(!switch_state);

    // make sure it is not armed anymore
    ASSERT_TRUE(!nts->is_armed());
}
