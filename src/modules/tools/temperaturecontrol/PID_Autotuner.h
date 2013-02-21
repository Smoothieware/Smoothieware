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
    void     on_idle(void*);

    TemperatureControl *t;

    double target_temperature;

    int cycle;
    bool output;
    bool last_output;
    StreamOutput *s;

    volatile bool tick;

    struct {
        double t_max;
        double t_min;
        int ticks_low;
        int ticks_high;
    } cycles[PID_AUTOTUNER_CYCLES];

    int bias, d;
};

#endif /* _PID_AUTOTUNE_H */
