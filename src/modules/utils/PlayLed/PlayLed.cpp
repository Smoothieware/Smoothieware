#include "PlayLed.h"

/*
 * LED indicator:
 * off   = not paused, nothing to do
 * flash = paused
 * on    = a block is being executed
 */

#include "PauseButton.h"
#include "modules/robot/Conveyor.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Pauser.h"
#include "checksumm.h"
#include "ConfigValue.h"


#define pause_led_pin_checksum      CHECKSUM("pause_led_pin")
#define play_led_pin_checksum       CHECKSUM("play_led_pin")
#define play_led_disable_checksum   CHECKSUM("play_led_disable")

PlayLed::PlayLed() {}

void PlayLed::on_module_loaded()
{
    if(THEKERNEL->config->value( play_led_disable_checksum )->by_default(false)->as_bool()) {
        delete this;
        return;
    }

    //register_for_event(ON_PLAY);
    //TODO: these two events happen in interrupt context and it's extremely important they don't last long. This should be done by checking the size of the queue once a second or something
    //register_for_event(ON_BLOCK_BEGIN);
    //register_for_event(ON_BLOCK_END);

    on_config_reload(this);

    THEKERNEL->slow_ticker->attach(4, this, &PlayLed::half_second_tick);
}

void PlayLed::on_config_reload(void *argument)
{
    string ledpin = "4.28!";

    ledpin = THEKERNEL->config->value( pause_led_pin_checksum )->by_default(ledpin)->as_string(); // check for pause_led_pin first
    ledpin = THEKERNEL->config->value( play_led_pin_checksum  )->by_default(ledpin)->as_string(); // override with play_led_pin if it's found

    led.from_string(ledpin)->as_output()->set(false);
}

void PlayLed::on_block_begin(void *argument)
{
    //led.set(true);
}

void PlayLed::on_block_end(void *argument)
{
    //led.set(false);
}

void PlayLed::on_play(void *argument)
{
    led.set(false);
}

uint32_t PlayLed::half_second_tick(uint32_t)
{
    if (THEKERNEL->pauser->paused())
        led.set(!led.get());
    else led.set(!THEKERNEL->conveyor->is_queue_empty());

    return 0;
}
