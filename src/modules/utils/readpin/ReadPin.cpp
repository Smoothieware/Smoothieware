#include "libs/Kernel.h"
#include "ReadPin.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "SlowTicker.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

using namespace std;

#define gcode_checksum               CHECKSUM("gcode")
#define pin_checksum                 CHECKSUM("pin")

ReadPin::ReadPin() {
}

ReadPin::ReadPin(uint16_t name) {
    name_checksum = name;
}

void ReadPin::on_module_loaded() {
    THEKERNEL->slow_ticker->attach(5, this, &ReadPin::pin_tick);
    register_for_event(ON_GCODE_RECEIVED);

    // Settings
    on_config_reload(this);
}

// Get config
void ReadPin::on_config_reload(void*) {
    my_pin.from_string(THEKERNEL->config->value(readpin_checksum, name_checksum, pin_checksum )->by_default("nc")->as_string())->as_input();
    if (my_pin.connected())
        my_pin.pull_none();
    auto gc = THEKERNEL->config->value(readpin_checksum, name_checksum, gcode_checksum)->by_default("")->as_string();
    my_gcode.reset(new Gcode(gc, nullptr)); // alas, no make_unique
}

void ReadPin::on_gcode_received(void* argument) {
    auto gcode = static_cast<Gcode*>(argument);
    if ((gcode->has_g && my_gcode->has_g && (gcode->g == my_gcode->g)) ||
        (gcode->has_m && my_gcode->has_m && (gcode->m == my_gcode->m))) {
        if (!my_pin.connected())
            THEKERNEL->streams->printf("Pin is not connected\n");
        else
            THEKERNEL->streams->printf("Pin %d.%d = %d\n", my_pin.port_number, my_pin.pin, state);
    }
}

// Check the state of the pin
// Note this is ISR so don't do anything nasty in here
uint32_t ReadPin::pin_tick(uint32_t) {
    state = my_pin.get();

    return 0;
}
