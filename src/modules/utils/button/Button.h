#ifndef BUTTON_H
#define BUTTON_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include <string>

#define button_checksum             CHECKSUM("button")
#define on_m_code_checksum          CHECKSUM("on_m_code")
#define off_m_code_checksum         CHECKSUM("off_m_code")
#define input_pin_checksum          CHECKSUM("input_pin")
#define toggle_checksum             CHECKSUM("toggle")
#define normal_state_checksum       CHECKSUM("normal_state")

class Button : public Module {
    public:
        Button(uint16_t name);

        void on_module_loaded();
        void on_config_reload(void* argument);
        uint32_t button_tick(uint32_t dummy);

        void send_gcode(string msg, StreamOutput* stream);

        bool       toggle;
        bool       button_state;
        bool       switch_state;
        uint16_t   name_checksum;
        std::string   on_m_code;
        std::string   off_m_code;
        Pin*       button;

        StreamOutput* dummy_stream;
};

#endif // BUTTON_H
