#ifndef _Leds_H
#define _Leds_H

#include "libs/Kernel.h"
#include "libs/Pin.h"

#define leds_checksum     CHECKSUM("leds")

typedef void (*Leds_handler)(Pin& pin, char event, void* data);


class Leds : public Module {
public:
    Leds();
    ~Leds();

    void on_module_loaded();
    void on_config_reload(void*);
    void on_post(void*);
    void on_sd_ok(void*);
    void on_main_loop(void*);
    void on_idle(void*);
    void on_gcode_received(void*);

    uint32_t half_second_tick(uint32_t);

private:
    Pin         pin_sdok;
    Pin         pin_gcode;
    Pin         pin_main;
    Pin         pin_idle;
    Pin         pin_play;

    Leds_handler handler_main;
    Leds_handler handler_idle;

    int16_t     counter_main;
    int16_t     counter_idle;
    int16_t     counter_gcode;
};

#endif /* _Leds_H */
