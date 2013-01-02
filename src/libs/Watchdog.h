#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <stdint.h>

typedef enum
{
    WDT_MRI,
    WDT_RESET,
} WDT_ACTION;

class Watchdog
{
public:
    Watchdog(uint32_t timeout, WDT_ACTION action);
    void feed();
};

#endif /* _WATCHDOG_H */
