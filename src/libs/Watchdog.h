#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include <stdint.h>

#include "Module.h"

typedef enum
{
    WDT_MRI,
    WDT_RESET,
} WDT_ACTION;

extern "C" {
    void __mriPlatform_EnteringDebuggerHook();
    void __mriPlatform_LeavingDebuggerHook();
}

class Watchdog : public Module
{
public:
    Watchdog(uint32_t timeout, WDT_ACTION action);
    void feed();

    void on_module_loaded();
    void on_idle(void*);
};

#endif /* _WATCHDOG_H */
