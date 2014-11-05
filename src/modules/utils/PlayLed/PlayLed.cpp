#include "PlayLed.h"

/*
 * LED indicator:
 * off   = not paused, nothing to do
 * slow flash = paused
 * fast flash = halted
 * on    = a block is being executed
 */

#include "PauseButton.h"
#include "modules/robot/Conveyor.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Pauser.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"

#define pause_led_pin_checksum      CHECKSUM("pause_led_pin")
#define play_led_pin_checksum       CHECKSUM("play_led_pin")
#define play_led_disable_checksum   CHECKSUM("play_led_disable")

PlayLed::PlayLed() {

    halted= false;
    cnt= 0;
}

void PlayLed::on_module_loaded()
{
    if(THEKERNEL->config->value( play_led_disable_checksum )->by_default(false)->as_bool()) {
        delete this;
        return;
    }

    on_config_reload(this);
    this->register_for_event(ON_HALT);
    THEKERNEL->slow_ticker->attach(12, this, &PlayLed::led_tick);
}

void PlayLed::on_config_reload(void *argument)
{
    string ledpin = "4.28!";

    ledpin = THEKERNEL->config->value( pause_led_pin_checksum )->by_default(ledpin)->as_string(); // check for pause_led_pin first
    ledpin = THEKERNEL->config->value( play_led_pin_checksum  )->by_default(ledpin)->as_string(); // override with play_led_pin if it's found

    led.from_string(ledpin)->as_output()->set(false);
}

uint32_t PlayLed::led_tick(uint32_t)
{
    if(this->halted) {
        led.set(!led.get());
        return 0;
    }

    if(++cnt >= 6) { // 6 ticks ~ 500ms
        cnt= 0;

        if (THEKERNEL->pauser->paused()) {
            led.set(!led.get());
        } else {
            led.set(!THEKERNEL->conveyor->is_queue_empty());
        }
    }

    return 0;
}

void PlayLed::on_halt(void *arg)
{
    this->halted= (arg == nullptr);
}
