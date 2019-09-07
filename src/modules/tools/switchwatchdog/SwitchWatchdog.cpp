
/*
    This implements a watchdog that can watch a given pin.
    When the pin goes high, a timer will start to count.
    Once timed out command <exec_command> will be run.
    Clearing the timer is done by pin low or sending the <clear_command> Gcode
*/

#include "SwitchWatchdog.h"
#include "Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"
#include "StreamOutputPool.h"
#include "StreamOutput.h"
#include "SerialMessage.h"
#include "utils.h"
#include "Gcode.h"

#include <algorithm>

#define switchwatchdog_checksum     CHECKSUM("switchwatchdog")
#define pin_checksum                CHECKSUM("pin")
#define timeout_checksum            CHECKSUM("timeout")
#define exec_command_checksum       CHECKSUM("exec_command")
#define clear_command_checksum      CHECKSUM("clear_command")
 
SwitchWatchdog::SwitchWatchdog(uint16_t name)
{
    this->timed_out     = false;
    this->command_sent  = false;
    this->name_checksum = name;
}

SwitchWatchdog::~SwitchWatchdog()
{
}

void SwitchWatchdog::on_module_loaded()
{
    //THEKERNEL->streams->printf("// SwitchWatchdog: init start\r\n");

    std::string pin_s = THEKERNEL->config->value(switchwatchdog_checksum, this->name_checksum, pin_checksum)->by_default("nc" )->as_string();
    this->pin.from_string( pin_s )->as_input();
    if(this->pin.connected()) {
        // fine!
        THEKERNEL->slow_ticker->attach( 100, this, &SwitchWatchdog::button_tick);
    } else {
        // free the module if not a valid configuration
        THEKERNEL->streams->printf("// SwitchWatchdog: failed to get pin '%s'\r\n", pin_s.c_str());
        delete this;
        return;
    }

    // timeout when no valid value causes a suspend
    configured_timeout = THEKERNEL->config->value(switchwatchdog_checksum, this->name_checksum, timeout_checksum)->by_default(10)->as_number();
    reset_timer();
    
    // this command will be executed on watchdog timeout
    this->exec_command = THEKERNEL->config->value(switchwatchdog_checksum, this->name_checksum, exec_command_checksum )->by_default("")->as_string();
    std::replace(exec_command.begin(), exec_command.end(), '_', ' '); // replace _ with space
    
    // this gcode will clear the watchdog
    std::string clear_command = THEKERNEL->config->value(switchwatchdog_checksum, this->name_checksum, clear_command_checksum )->by_default("")->as_string();
    
    // Set the on/off command codes, Use GCode to do the parsing
    clear_command_letter = 0;

    if(!clear_command.empty()) {
        Gcode gc(clear_command , NULL);
        if(gc.has_g) {
            clear_command_letter = 'G';
            clear_command_code   = gc.g;
        } else if(gc.has_m) {
            clear_command_letter = 'M';
            clear_command_code   = gc.m;
        }
    }
    
    // register event-handlers
    register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    
    //THEKERNEL->streams->printf("// SwitchWatchdog: init done\n");
}

bool SwitchWatchdog::match_clear_gcode(const Gcode *gcode) const
{
    bool b= ((clear_command_letter == 'M' && gcode->has_m && gcode->m == clear_command_code) ||
            (clear_command_letter == 'G' && gcode->has_g && gcode->g == clear_command_code));

    return (b);
}

void SwitchWatchdog::send_command(std::string msg, StreamOutput *stream)
{
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}


void SwitchWatchdog::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
 
    // check if this resets the timeout
    if(match_clear_gcode(gcode)) {
        // clear timeout 
        reset_timer();
        THEKERNEL->streams->printf("// SwitchWatchdog: CLEAR GCODE received... cleared timer\r\n");
    }
    
    // check if timed out:
    if (timed_out) {
        // send command and stay in timed out mode  until flag is cleared
        if (!command_sent) {
            if(!this->exec_command.empty()) this->send_command( this->exec_command, &(StreamOutput::NullStream) );
            THEKERNEL->streams->printf("// SwitchWatchdog: TIMEOUT! sending command\r\n");
            command_sent = true;
        }
    }
}

// needed to detect when we resume
void SwitchWatchdog::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);
    string possible_command = new_message.message;
    string cmd = shift_parameter(possible_command);
    if(cmd == "resume" || cmd == "M601") {
        if (timer == 0) {
            THEKERNEL->streams->printf("// SwitchWatchdog: RESUMING while in button HIGH and in timeout! FIX ISSUE!\r\n");
            if(!this->exec_command.empty()) this->send_command( this->exec_command, &(StreamOutput::NullStream) );
        }
    }
}


void SwitchWatchdog::on_second_tick(void *argument)
{
    if(timed_out) return;

    timer--;
    
    //THEKERNEL->streams->printf("// SwitchWatchdog: TIMER = %d\r\n", timer);
    
    if (timer == 0) {
        // too late, alarm!
        timed_out = true;
        //THEKERNEL->streams->printf("// SwitchWatchdog: TIMED OUT!\r\n");
    }
    
    if (!pin.get()) {
        // pin state is good, reset timer
        reset_timer();
    }
}

void SwitchWatchdog::reset_timer(void)
{
    timer        = configured_timeout;
    timed_out    = false;
    command_sent = false;
}

uint32_t SwitchWatchdog::button_tick(uint32_t dummy)
{
    if(!pin.get()) {
        // pin state is good, reset timer
        reset_timer();
    }

    return 0;
}


