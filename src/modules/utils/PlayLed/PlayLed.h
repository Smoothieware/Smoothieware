#ifndef _PLAYLED_H
#define _PLAYLED_H

#include "libs/Kernel.h"
#include "libs/Pin.h"


class PlayLed : public Module
{
public:
    PlayLed();

    void on_module_loaded(void);
    void on_config_reload(void *);

private:
    uint32_t led_tick(uint32_t);
    Pin  led;
    struct {
        uint8_t cnt:4;
    };
};

#endif /* _PLAYLED_H */
