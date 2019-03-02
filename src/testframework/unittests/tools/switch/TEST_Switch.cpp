#include "Kernel.h"
#include "checksumm.h"
#include "utils.h"
#include "Test_kernel.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "Gcode.h"
#include "Switch.h"
#include "mri.h"

#include <stdio.h>
#include <memory>

#include "easyunit/test.h"

// this declares any global variables the test needs
DECLARE(Switch)
  Switch *ts;
  uint16_t switch_id;
END_DECLARE

// called before each test
SETUP(Switch)
{
    // create the module we want to test
    switch_id= get_checksum("fan");
    ts = new Switch(switch_id);
    //printf("...Setup Switch\n");
}

// called after each test
TEARDOWN(Switch)
{
    // delete the module
    delete ts;

    // have kernel reset to a clean state
    test_kernel_teardown();

    //printf("...Teardown Switch\n");
}

// define various configs here, these are in the same formate they would appeat in the config file (comments removed for clarity)
const static char switch_config[]= "\
switch.fan.enable true \n\
switch.fan.input_on_command M106 \n\
switch.fan.input_off_command M107 \n\
switch.fan.output_pin 2.6  \n\
switch.fan.output_type digital  \n\
";

static bool get_switch_state(Switch *ts, struct pad_switch& s)
{
    // inquire if the switch is on or off (simulate an ON_GET_PUBLIC_DATA)
    PublicDataRequest pdr(switch_checksum, fan_checksum, 0);
    pdr.set_data_ptr(&s, false); // caller providing storage
    ts->on_get_public_data(&pdr);
    return(pdr.is_taken() && !pdr.has_returned_data());
}

TESTF(Switch,set_on_off_with_gcode)
{
    // load config with required settings for this test
    test_kernel_setup_config(switch_config, &switch_config[sizeof(switch_config)]);

    // make module load the config
    ts->on_config_reload(nullptr);

    // make sure switch starts off
    struct pad_switch s;
    ASSERT_TRUE(get_switch_state(ts, s));
    ASSERT_EQUALS(s.name, switch_id);
    ASSERT_TRUE(!s.state);

    // if we want to enter the debugger here
    //__debugbreak();

    // now turn it on
    Gcode gc1("M106", (StreamOutput *)THEKERNEL->serial);
    ts->on_gcode_received(&gc1);
    ASSERT_TRUE(get_switch_state(ts, s));
    ASSERT_TRUE(s.state);

    // now turn it off
    Gcode gc2("M107", (StreamOutput *)THEKERNEL->serial);
    ts->on_gcode_received(&gc2);
    ASSERT_TRUE(get_switch_state(ts, s));
    ASSERT_TRUE(!s.state);
}
