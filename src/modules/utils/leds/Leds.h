#ifndef _Leds_H
#define _Leds_H

#include "libs/Kernel.h"
#include "libs/Pin.h"
#include <string>

#define leds_checksum     CHECKSUM("leds")
#define post_checksum     CHECKSUM("post")
#define sdok_checksum     CHECKSUM("sdok")
#define END_OF_POST       0xFF

typedef void (*Leds_handler)(Pin& pin, char event, void* data);


class Leds : public Module {
public:
    Leds();
    ~Leds();

    void on_module_loaded();
    void on_config_reload(void*);
    void on_main_loop(void*);
    void on_idle(void*);
    void on_gcode_received(void*);
    void on_set_public_data(void*);

    uint32_t half_second_tick(uint32_t);

private:
    void show_post(int post);

    Pin         pin_sdok;
    Pin         pin_gcode;
    Pin         pin_main;
    Pin         pin_idle;
    Pin         pin_play;
    std::string pins_post;

    Leds_handler handler_main;
    Leds_handler handler_idle;

    int16_t     counter_main;
    int16_t     counter_idle;
    int16_t     counter_gcode;
};

#endif /* _Leds_H */
