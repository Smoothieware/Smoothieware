#include "Endstop.h"

#include "Kernel.h"

#define pin_checksum               CHECKSUM("pin")
#define stopping_distance_checksum CHECKSUM("stopping_distance")
#define debounce_count_checksum    CHECKSUM("debounce_count")
#define position_checksum          CHECKSUM("position")
#define name_checksum              CHECKSUM("name")

typedef struct
{
    const uint16_t csum;
    const char     name[6];
} name_pair;

const name_pair cartesian_names[6] = {
    { CHECKSUM("x_min"), "x_min" },
    { CHECKSUM("x_max"), "x_max" },
    { CHECKSUM("y_min"), "y_min" },
    { CHECKSUM("y_max"), "y_max" },
    { CHECKSUM("z_min"), "z_min" },
    { CHECKSUM("z_max"), "z_max" }
};
const name_pair actuator_names[6] = {
    { CHECKSUM("alpha_min"), "a_min" },
    { CHECKSUM("alpha_max"), "a_max" },
    { CHECKSUM("beta_min") , "b_min" },
    { CHECKSUM("beta_max") , "b_max" },
    { CHECKSUM("gamma_min"), "c_min" },
    { CHECKSUM("gamma_max"), "c_max" }
};

Endstop::Endstop(uint16_t myname_checksum)
{
    this->myname_checksum = myname_checksum;

    type = ENDSTOP_TYPE_UNCOMMITTED;

    debounce = last_change_count = 0;
    state = false;
    debounce_count = 8;

    name = "?";
}

void Endstop::on_module_loaded()
{
    register_for_event(ON_CONFIG_RELOAD);

    on_config_reload(this);

    THEKERNEL->serial->printf("E %u %s\n", name_checksum, name);
}

void Endstop::on_config_reload(void*)
{
    pin.from_string(    THEKERNEL->config->value(endstop_checksum, myname_checksum, pin_checksum              )->by_default("nc")->as_string() )->as_input();

    stopping_distance = THEKERNEL->config->value(endstop_checksum, myname_checksum, stopping_distance_checksum)->by_default(0.5F)->as_number();
    debounce_count    = THEKERNEL->config->value(endstop_checksum, myname_checksum, debounce_count_checksum   )->by_default(8   )->as_number();
    position          = THEKERNEL->config->value(endstop_checksum, myname_checksum, position_checksum         )->by_default(0.0F)->as_number();

    /*
     * find out what sort of endstop this is
     */
    type = ENDSTOP_TYPE_UNCOMMITTED;
    for (int i = 0; i < 6; i++)
    {
        if (myname_checksum == cartesian_names[i].csum)
        {
            type = ENDSTOP_TYPE_CARTESIAN;
            name = cartesian_names[i].name;
            type_index = i;
            break;
        }
        else if (myname_checksum == actuator_names[i].csum)
        {
            type = ENDSTOP_TYPE_ACTUATOR;
            name = actuator_names[i].name;
            type_index = i;
            break;
        }
    }

    // accept user-override name if present
//     name              = THEKERNEL->config->value(endstop_checksum, myname_checksum, name_checksum             )->by_default(name)->as_string().c_str();
}

uint8_t Endstop::axis()
{
    return type_index >> 1;
}

Endstop_Direction_e Endstop::dir()
{
    return (type_index & 1)?ENDSTOP_DIR_MAX:ENDSTOP_DIR_MIN;
}

void Endstop::poll()
{
    bool instantaneous_reading = pin.get();

    // smoothing & denoise
    if (instantaneous_reading && (debounce < debounce_count))
        debounce++;
    else if ((!instantaneous_reading) && (debounce > 0))
        debounce--;

    // sympathetic oscillation mitigation
    if (last_change_count >= debounce_count)
    {
        bool newstate = state;

        // hysteresis
        if (debounce > ((debounce_count >> 1) + (debounce_count >> 2))) // 1/2 + 1/4 = 3/4
            newstate = true;
        else if (debounce < (debounce_count >> 2))
            newstate = false;

        if (newstate != state)
        {
            state = newstate;
            last_change_count = 0;
        }
    }
    else
        last_change_count++;
}

bool Endstop::asserted()
{
    return state;
}
