#include "libs/Kernel.h"
#include "KillButton.h"
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

#define pause_button_enable_checksum CHECKSUM("pause_button_enable")
#define kill_button_enable_checksum  CHECKSUM("kill_button_enable")
#define pause_button_pin_checksum    CHECKSUM("pause_button_pin")
#define kill_button_pin_checksum     CHECKSUM("kill_button_pin")

KillButton::KillButton()
{
    this->button_state = true;
    this->killed = false;
    this->do_kill= false;
}

void KillButton::on_module_loaded()
{
    bool pause_enable = THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool(); // @deprecated
    this->kill_enable = pause_enable || THEKERNEL->config->value( kill_button_enable_checksum )->by_default(false)->as_bool();
    if(!this->kill_enable) {
        delete this;
        return;
    }

    Pin pause_button;
    pause_button.from_string( THEKERNEL->config->value( pause_button_pin_checksum )->by_default("2.12")->as_string())->as_input(); // @DEPRECATED
    this->kill_button.from_string( THEKERNEL->config->value( kill_button_pin_checksum )->by_default("nc")->as_string())->as_input();

    if(!this->kill_button.connected() && pause_button.connected()) {
        // use pause button for kill button if kill button not specifically defined
        this->kill_button = pause_button;
    }

    if(!this->kill_button.connected()) {
        delete this;
        return;
    }

    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_IDLE);
    THEKERNEL->slow_ticker->attach( 10, this, &KillButton::button_tick );
}

void KillButton::on_idle(void *argument)
{
    if(do_kill) {
        do_kill= false;
        THEKERNEL->call_event(ON_HALT, nullptr);
        THEKERNEL->streams->printf("Kill button pressed - reset or M999 to continue\r\n");
    }
}

//TODO: Make this use InterruptIn
//Check the state of the button and act accordingly based on current pause state
// Note this is ISR so don't do anything nasty in here
uint32_t KillButton::button_tick(uint32_t dummy)
{
    if(!this->killed && this->kill_enable && this->kill_button.connected() && !this->kill_button.get()) {
        this->killed = true;
        // we can't call this in ISR, and we need to block on_main_loop so do it in on_idle
        // THEKERNEL->call_event(ON_HALT);
        this->do_kill= true;
    }

    return 0;
}

// When a new line is received, check if it is a command, and if it is, act upon it
void KillButton::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    if(this->killed && new_message.message == "M999") {
        this->killed= false;
        return;
    }
}

