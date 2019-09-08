#pragma once

#include "Module.h"
#include "Pin.h"

#include <stdint.h>
#include <atomic>
#include <string>

class Gcode;
class StreamOutput;

class SwitchWatchdog: public Module
{
public:
    SwitchWatchdog(uint16_t name);
    ~SwitchWatchdog();
    void on_module_loaded();
    void on_second_tick(void* argument);
    void on_gcode_received(void *argument);
    void on_console_line_received( void *argument );

private:
    uint32_t button_tick(uint32_t dummy);
    void reset_timer(void);
    void send_command(std::string msg, StreamOutput *stream);
    void init_command(uint16_t checksum, uint16_t *code, char *letter);
    bool match_gcode(const Gcode *gcode, uint16_t code, char letter) const;
    /*void on_pin_rise();
    void check_encoder();
    void send_command(std::string msg, StreamOutput *stream);
    float get_emove();

    mbed::InterruptIn *encoder_pin{0};
    float e_last_moved{0};
    std::atomic_uint pulses{0};
    float pulses_per_mm{0};

    struct {
        bool filament_out_alarm:1;
        bool bulge_detected:1;
        bool suspended:1;
        bool active:1;
    };*/
    
    std::string    exec_command;
    std::string    clear_command;
    
    uint8_t timer{0};
    uint8_t configured_timeout{10};
    
    Pin pin;
    
    uint16_t  enable_command_code;
    uint16_t  disable_command_code;
    uint16_t  clear_command_code;
    char      enable_command_letter;
    char      disable_command_letter;
    char      clear_command_letter;
    
    struct {
            uint16_t  name_checksum:16;
            bool      activated:1;
            bool      timed_out:1;
            bool      command_sent:1;
    };
};
