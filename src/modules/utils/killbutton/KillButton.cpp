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
#define toggle_checksum              CHECKSUM("kill_button_toggle_enable")
#define unkill_checksum              CHECKSUM("unkill_enable")
#define kill_button_pin_checksum     CHECKSUM("kill_button_pin")
#define poll_frequency_checksum      CHECKSUM("kill_button_poll_frequency")

KillButton::KillButton()
{
    this->state= IDLE;
    this->estop_still_pressed= false;
}

void KillButton::on_module_loaded()
{
    // @DEPRECATED
    bool pause_enable = THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool();
    bool kill_enable = pause_enable || THEKERNEL->config->value( kill_button_enable_checksum )->by_default(false)->as_bool();
    if(!kill_enable) {
        delete this;
        return;
    }
    this->kill_button.from_string( THEKERNEL->config->value( kill_button_pin_checksum )->by_default("2.12")->as_string())->as_input();

    if(!this->kill_button.connected()) {
        delete this;
        return;
    }

    this->unkill_enable = THEKERNEL->config->value( unkill_checksum )->by_default(true)->as_bool();
    this->toggle_enable = THEKERNEL->config->value( toggle_checksum )->by_default(false)->as_bool();

    this->register_for_event(ON_IDLE);

    this->poll_frequency = THEKERNEL->config->value( poll_frequency_checksum )->by_default(5)->as_number();
    THEKERNEL->slow_ticker->attach( this->poll_frequency, this, &KillButton::button_tick );
}

void KillButton::on_idle(void *argument)
{
    if(state == KILL_BUTTON_DOWN) {
        if(!THEKERNEL->is_halted()) {
            THEKERNEL->call_event(ON_HALT, nullptr);
            if(estop_still_pressed) {
                THEKERNEL->streams->printf("WARNING: ESTOP is still latched, unlatch ESTOP to clear HALT\n");
                estop_still_pressed= false;
            }else{
                THEKERNEL->streams->printf("ALARM: Kill button pressed - reset, $X or M999 to clear HALT\n");
            }
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
// If in toggle mode (locking estop) then button down will kill, and button up will unkill if unkill is enabled
// otherwise it will look for a 2 second press on the kill button to unkill if unkill is set
uint32_t KillButton::button_tick(uint32_t dummy)
{
    bool killed= THEKERNEL->is_halted();

    switch(state) {
            case IDLE:
                if(!this->kill_button.get()) state= KILL_BUTTON_DOWN;
                else if(unkill_enable && !toggle_enable && killed) state= KILLED_BUTTON_UP; // allow kill button to unkill if kill was created from some other source
                break;
            case KILL_BUTTON_DOWN:
                if(killed) state= KILLED_BUTTON_DOWN;
                break;
            case KILLED_BUTTON_DOWN:
                if (this->kill_button.get()) {
                    state= KILLED_BUTTON_UP;
                } else if ((toggle_enable) && (!killed)) {
                    // button is still pressed but the halted state was left
                    // re-trigger the halted state
                    state= KILL_BUTTON_DOWN;
                    estop_still_pressed= true;
                }
                break;
            case KILLED_BUTTON_UP:
                if(!killed) state= IDLE;
                if(unkill_enable) {
                    if(toggle_enable) state= UNKILL_FIRE; // if toggle is enabled and button is released then we unkill
                    else if(!this->kill_button.get()) state= UNKILL_BUTTON_DOWN; // wait for button to be pressed to go into next state for timing start
                }
                break;
            case UNKILL_BUTTON_DOWN:
                unkill_timer= 0;
                state= UNKILL_TIMING_BUTTON_DOWN;
                break;
            case UNKILL_TIMING_BUTTON_DOWN:
                if(++unkill_timer > this->poll_frequency*2) state= UNKILL_FIRE;
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
