#include "Leds.h"

#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "utils.h"
#include "SlowTicker.h"
#include "Pauser.h"
#include "modules/robot/Conveyor.h"

#include "wait_api.h"

#define enable_checksum         CHECKSUM("enable")
#define leds_disable_checksum   CHECKSUM("leds_disable")

#define pins_post_checksum      CHECKSUM("pins_post")
#define pin_gcode_checksum      CHECKSUM("pin_gcode")
#define pin_main_checksum       CHECKSUM("pin_main")
#define pin_idle_checksum       CHECKSUM("pin_idle")
#define pin_sdok_checksum       CHECKSUM("pin_sdok")
#define pin_play_checksum       CHECKSUM("pin_play")

#define MODE_BLINK              'b'
#define MODE_DIMMED             'd'

#define PIN_LED1                "1.18"
#define PIN_LED2                "1.19"
#define PIN_LED3                "1.20"
#define PIN_LED4                "1.21"
#define PIN_LED5                "4.28!"

#define PINS_POST_DEFAULT       PIN_LED1 " " PIN_LED2 " " PIN_LED3 " " PIN_LED4 " " PIN_LED5

static void off(Pin& pin, char event, void* data) {
}

static void blink(Pin& pin, char event, void* data) {
    int& counter = *(int*)data;
    switch(event) {
    case 't':
        pin.set( counter++ & 0x1000 );
        break;
    }
}

static void dimmed(Pin& pin, char event, void* data) {
    int& counter = *(int*)data;
    switch(event) {
    case 't':
        pin.set( ! (counter++ & 0x0700) );
        break;
    }
}

static void flash(Pin& pin, char event, void* data) {
    int& counter = *(int*)data;
    switch(event) {
    case 't':
        counter = 1;
        pin.set(true);
        break;
    case 'i':
        if(counter > 0) {
            counter++;
            if(counter > 0x0400) {
                counter = 0;
                pin.set(false);
            }
        }
        break;
    }
}

static void config_pin(Pin& pin, uint16_t checksum, string default_pin_setup, string defaults, Leds_handler* handler = 0, int16_t* counter = 0) {

    string config = THEKERNEL->config->value( leds_checksum, checksum  )->by_default(defaults)->as_string();

    string pin_setup;
    while(config.size()) {
        string param = shift_parameter(config);
        if( ! param.size() )
            continue;
        if(handler && param == "off")
            *handler = off;
        else
        if(handler && param == "blink")
            *handler = blink;
        else
        if(handler && param == "dimmed")
            *handler = dimmed;
        else
            pin_setup = param;
    }
    if(counter)
        *counter = 0;
    if(handler && *handler == off)
        pin_setup = "";
    else
    if( ! pin_setup.size() )
        pin_setup = default_pin_setup;
    if(pin_setup.size())
        pin.from_string(pin_setup)->as_output()->set(false);
}

#define CONFIG_PIN___(name,default_pin_setup,defaults) \
    config_pin(pin_##name, pin_##name##_checksum, default_pin_setup, defaults)

#define CONFIG_PIN_HC(name,default_pin_setup,defaults) \
    config_pin(pin_##name, pin_##name##_checksum, default_pin_setup, defaults, &handler_##name, &counter_##name)

#define CONFIG_PIN__C(name,default_pin_setup,defaults) \
    config_pin(pin_##name, pin_##name##_checksum, default_pin_setup, defaults, 0, &counter_##name)


Leds::Leds()  {}
Leds::~Leds() {}

void Leds::on_module_loaded()
{
    // old config leds_disable
    bool use_leds = ! THEKERNEL->config->value( leds_disable_checksum )->by_default(false)->as_bool();
         use_leds =   THEKERNEL->config->value( leds_checksum, enable_checksum )->by_default( use_leds )->as_bool();

    // Exit if this module is not enabled
    if ( ! use_leds ) {
        delete this;
        return;
    }

    // Settings
    this->on_config_reload(this);

    // register events after initializing scheme
    register_for_event(ON_POST);
    register_for_event(ON_SD_OK);
    register_for_event(ON_MAIN_LOOP);
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_RECEIVED);

    THEKERNEL->slow_ticker->attach(4, this, &Leds::half_second_tick);
}

void Leds::on_config_reload(void* argument)
{
    CONFIG_PIN__C( gcode, PIN_LED1, ""       );
    CONFIG_PIN_HC( main,  PIN_LED2, "dimmed" );
    CONFIG_PIN_HC( idle,  PIN_LED3, "dimmed" );
    CONFIG_PIN___( sdok,  PIN_LED4, ""       );
    CONFIG_PIN___( play,  PIN_LED5, ""       );
}

void Leds::on_post(void* argument)         {
    if(argument) {
        int post = *(int*)argument;
        // scan comma separated config string for pin descriptions
        // cpu time doesn't matter while booting
        string pins_post = THEKERNEL->config
                              ->value( leds_checksum, pins_post_checksum )
                              ->by_default(PINS_POST_DEFAULT)
                              ->as_string();
        int16_t mask = 1;
        while(pins_post.size()) {
            Pin pin;
            string pin_bit = shift_parameter(pins_post);
            pin.from_string(pin_bit)->as_output();
            pin.set(post & mask);
            mask = (mask << 1);
        }
    wait_ms(100);
    }
}

void Leds::on_sd_ok(void* argument)             {
    if(pin_sdok.connected())
        pin_sdok.set(true);
}

void Leds::on_main_loop(void* argument)         {
    if(pin_main.connected()) {
        handler_main(pin_main, 't', &counter_main);
    }
}

void Leds::on_idle(void* argument)              {
    if(pin_idle.connected()) {
        handler_idle(pin_idle, 't', &counter_idle);
    }
    if(pin_gcode.connected()) {
        flash(pin_gcode, 'i', &counter_gcode);
    }
}

void Leds::on_gcode_received(void* argument)    {
    if(pin_gcode.connected()) {
        flash(pin_gcode, 't', &counter_gcode);
    }
}

uint32_t Leds::half_second_tick(uint32_t)
{
    if(pin_play.connected()) {
        if (THEKERNEL->pauser->paused())
            pin_play.set( ! pin_play.get() );
        else
            pin_play.set( ! THEKERNEL->conveyor->is_queue_empty() );
    }
    return 0;
}
