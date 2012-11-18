#ifndef BUTTON_H
#define BUTTON_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include <string>

#define button_checksum        5709
#define on_m_code_checksum     29094
#define off_m_code_checksum    14853
#define input_pin_checksum     21209
#define toggle_checksum        63876

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
        uint16_t   on_m_code;
        uint16_t   off_m_code;
        Pin*       button;

        StreamOutput* dummy_stream;
};

#endif // BUTTON_H
