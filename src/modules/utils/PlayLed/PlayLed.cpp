#include "PlayLed.h"

/*
 * LED indicator:
 * off   = not paused, nothing to do
 * flash = paused
 * on    = a block is being executed
 */

#include "PauseButton.h"

PlayLed::PlayLed(){}

void PlayLed::on_module_loaded()
{
    register_for_event(ON_CONFIG_RELOAD);

    register_for_event(ON_PLAY);
    register_for_event(ON_BLOCK_BEGIN);
    register_for_event(ON_BLOCK_END);

    on_config_reload(this);

    kernel->slow_ticker->attach(4, this, &PlayLed::half_second_tick);
}

void PlayLed::on_config_reload(void* argument)
{
    string ledpin = "4.28!";

    ledpin = kernel->config->value( pause_led_pin_checksum )->by_default(ledpin)->as_string(); // check for pause_led_pin first
    ledpin = kernel->config->value( play_led_pin_checksum  )->by_default(ledpin)->as_string(); // override with play_led_pin if it's found

    led.from_string(ledpin)->as_output()->set(false);
}

void PlayLed::on_block_begin(void* argument)
{
    led.set(true);
}

void PlayLed::on_block_end(void* argument)
{
    led.set(false);
}

void PlayLed::on_play(void* argument)
{
    led.set(false);
}

uint32_t PlayLed::half_second_tick(uint32_t)
{
    if (kernel->pauser->paused())
        led.set(!led.get());

    return 0;
}
