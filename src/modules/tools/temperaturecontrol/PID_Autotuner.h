#ifndef _PID_AUTOTUNE_H
#define _PID_AUTOTUNE_H

#include <stdint.h>

#include "Module.h"
#include "TemperatureControl.h"
#include "StreamOutput.h"

#define PID_AUTOTUNER_CYCLES 8

class PID_Autotuner : public Module
{
public:
             PID_Autotuner();
    void     begin(TemperatureControl*, double, StreamOutput*);

    void     on_module_loaded(void);
    uint32_t on_tick(uint32_t);

    TemperatureControl *t;

    int cycle;
    bool last_output;
    StreamOutput *s;

    struct {
        double t_max;
        double t_min;
        int ticks;
    } cycles[PID_AUTOTUNER_CYCLES];
};

#endif /* _PID_AUTOTUNE_H */
