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
    void on_play(         void *);
    void on_block_begin(  void *);
    void on_block_end(    void *);
    uint32_t half_second_tick(uint32_t);

    Pin  led;
};

#endif /* _PLAYLED_H */
