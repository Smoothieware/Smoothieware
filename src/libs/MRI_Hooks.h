#ifndef _MRI_HOOKS_H
#define _MRI_HOOKS_H

extern "C" {
    void __mriPlatform_EnteringDebuggerHook();
    void __mriPlatform_LeavingDebuggerHook();

    void set_high_on_debug(int port, int pin);
    void set_low_on_debug(int port, int pin);
}

#endif /* _MRI_HOOKS_H */
