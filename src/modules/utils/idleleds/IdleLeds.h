#ifndef _IdleLeds_H
#define _IdleLeds_H

#include "libs/Kernel.h"
#include "libs/Pin.h"

class IdleLeds : public Module {
public:
    IdleLeds();

    void on_module_loaded(void);
    void on_config_reload(void*);
    void on_main_loop(void*);
    void on_idle(void*);

    int8_t      led_main;
    int16_t     counter_main;

    int8_t      led_idle;
    int16_t     counter_idle;
};

#endif /* _IdleLeds_H */
