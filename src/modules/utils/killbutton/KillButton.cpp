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
#define unkill_checksum              CHECKSUM("unkill_enable")
#define pause_button_pin_checksum    CHECKSUM("pause_button_pin")
#define kill_button_pin_checksum     CHECKSUM("kill_button_pin")

KillButton::KillButton()
{
    this->state= IDLE;
}

void KillButton::on_module_loaded()
{
    bool pause_enable = THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool(); // @deprecated
    bool kill_enable = pause_enable || THEKERNEL->config->value( kill_button_enable_checksum )->by_default(false)->as_bool();
    if(!kill_enable) {
        delete this;
        return;
    }
    this->unkill_enable = THEKERNEL->config->value( unkill_checksum )->by_default(true)->as_bool();

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

    this->register_for_event(ON_IDLE);
    THEKERNEL->slow_ticker->attach( 5, this, &KillButton::button_tick );
}

void KillButton::on_idle(void *argument)
{
    if(state == KILL_BUTTON_DOWN) {
        if(!THEKERNEL->is_halted()) {
            THEKERNEL->call_event(ON_HALT, nullptr);
            THEKERNEL->streams->printf("Kill button pressed - reset or M999 to continue\r\n");
        }

    }else if(state == UNKILL_FIRE) {
        if(THEKERNEL->is_halted()) {
            THEKERNEL->call_event(ON_HALT, (void *)1); // clears on_halt
            THEKERNEL->streams->printf("UnKill button pressed Halt cleared\r\n");
        }
    }
}

// Check the state of the button and act accordingly using the following FSM
// Note this is ISR so don't do anything nasty in here
uint32_t KillButton::button_tick(uint32_t dummy)
{
    bool killed= THEKERNEL->is_halted();

    switch(state) {
            case IDLE:
                if(!this->kill_button.get()) state= KILL_BUTTON_DOWN;
                else if(unkill_enable && killed) state= KILLED_BUTTON_UP; // allow kill button to unkill if kill was created fromsome other source
                break;
            case KILL_BUTTON_DOWN:
                if(killed) state= KILLED_BUTTON_DOWN;
                break;
            case KILLED_BUTTON_DOWN:
                if(this->kill_button.get()) state= KILLED_BUTTON_UP;
                break;
            case KILLED_BUTTON_UP:
                if(!killed) state= IDLE;
                else if(unkill_enable && !this->kill_button.get()) state= UNKILL_BUTTON_DOWN;
                break;
            case UNKILL_BUTTON_DOWN:
                unkill_timer= 0;
                state= UNKILL_TIMING_BUTTON_DOWN;
                break;
            case UNKILL_TIMING_BUTTON_DOWN:
                if(++unkill_timer > 5*2) state= UNKILL_FIRE;
                else if(this->kill_button.get()) unkill_timer= 0;
                if(!killed) state= IDLE;
                break;
            case UNKILL_FIRE:
                 if(!killed) state= UNKILLED_BUTTON_DOWN;
                 break;
            case UNKILLED_BUTTON_DOWN:
                if(this->kill_button.get()) state= IDLE;
                break;
    }

    return 0;
}
