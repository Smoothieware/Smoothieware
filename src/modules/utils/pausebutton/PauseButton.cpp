#include "libs/Kernel.h"
#include "PauseButton.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "SlowTicker.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "Pauser.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

using namespace std;

#define pause_button_enable_checksum CHECKSUM("pause_button_enable")
#define kill_button_enable_checksum  CHECKSUM("kill_button_enable")
#define pause_button_pin_checksum    CHECKSUM("pause_button_pin")
#define kill_button_pin_checksum     CHECKSUM("kill_button_pin")

PauseButton::PauseButton()
{
    this->button_state = true;
    this->killed = false;
    this->do_kill= false;
}

void PauseButton::on_module_loaded()
{
    this->pause_enable = THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool();
    this->kill_enable  = THEKERNEL->config->value( kill_button_enable_checksum )->by_default(false)->as_bool();
    this->pause_button.from_string( THEKERNEL->config->value( pause_button_pin_checksum )->by_default("2.12")->as_string())->as_input();
    this->kill_button.from_string( THEKERNEL->config->value( kill_button_pin_checksum )->by_default("nc")->as_string())->as_input();

    if(this->kill_enable && this->kill_button.connected() && pause_button.equals(kill_button)) {
        // kill button takes priority
        this->pause_enable = false;

    } else if(this->kill_enable && !this->kill_button.connected() && !this->pause_enable && pause_button.connected()) {
        // use pause button for kill button if kill buttin not specifically defined
        this->kill_button = this->pause_button;
    }

    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);

    if( (this->pause_enable && this->pause_button.connected()) || (this->kill_enable && this->kill_button.connected()) ) {
        THEKERNEL->slow_ticker->attach( 10, this, &PauseButton::button_tick );
        this->register_for_event(ON_IDLE);
    }
}

void PauseButton::on_idle(void *argument)
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
uint32_t PauseButton::button_tick(uint32_t dummy)
{
    // If pause button changed
    if(this->pause_enable && this->pause_button.connected()) {
        bool newstate = this->pause_button.get();
        if(this->button_state != newstate) {
            this->button_state = newstate;
            // If button pressed
            if( this->button_state ) {
                if( THEKERNEL->pauser->paused() ) {
                    THEKERNEL->pauser->release();
                } else {
                    THEKERNEL->pauser->take();
                }
            }
        }
    }

    if(!this->killed && this->kill_enable && this->kill_button.connected() && !this->kill_button.get()) {
        this->killed = true;
        // we can't call this in ISR, and we need to block on_main_loop so do it in on_idle
        // THEKERNEL->call_event(ON_HALT);
        this->do_kill= true;
    }

    return 0;
}

// When a new line is received, check if it is a command, and if it is, act upon it
void PauseButton::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    if(this->killed && new_message.message == "M999") {
        this->killed= false;
        return;
    }

    // ignore comments and blank lines and if this is a G code then also ignore it
    char first_char = new_message.message[0];
    if(strchr(";( \n\rGMTN", first_char) != NULL) return;

    string cmd = shift_parameter(new_message.message);

    if (cmd == "freeze") {
        if( !THEKERNEL->pauser->paused() ) {
            THEKERNEL->pauser->take();
        }

    } else if (cmd == "unfreeze") {
        if( THEKERNEL->pauser->paused() ) {
            THEKERNEL->pauser->release();
        }
    }
}

