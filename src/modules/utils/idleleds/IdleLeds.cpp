#include "IdleLeds.h"

#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/gpio.h"

#define idleleds_checksum CHECKSUM("idleleds")
#define enable_checksum   CHECKSUM("enable")
#define led_main_checksum CHECKSUM("led_main")
#define led_idle_checksum CHECKSUM("led_idle")

extern GPIO leds[];

IdleLeds::IdleLeds(){}

void IdleLeds::on_module_loaded()
{
    // Exit if this module is not enabled
    if ( !THEKERNEL->config->value( idleleds_checksum, enable_checksum )->by_default(true)->as_bool() ) {
        delete this;
        return;
    }

    register_for_event(ON_CONFIG_RELOAD);
    register_for_event(ON_IDLE);
    register_for_event(ON_MAIN_LOOP);

    on_config_reload(this);
}

void IdleLeds::on_config_reload(void* argument)
{
    led_main = THEKERNEL->config->value( idleleds_checksum, led_main_checksum )->by_default(2)->as_int() - 1;
    led_idle = THEKERNEL->config->value( idleleds_checksum, led_idle_checksum )->by_default(3)->as_int() - 1;

    counter_main = 0;
    counter_idle = 0;
}

void IdleLeds::on_main_loop(void* argument)
{
    if(led_main >= 0) leds[led_main]= (counter_main++ & 0x1000) ? 1 : 0;
}

void IdleLeds::on_idle(void* argument)
{
    if(led_idle >= 0) leds[led_idle]= (counter_idle++ & 0x1000) ? 1 : 0;
}
