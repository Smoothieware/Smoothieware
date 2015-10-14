/**
 * Based on https://github.com/br3ttb/Arduino-PID-AutoTune-Library
 */

#ifndef _PID_AUTOTUNE_H
#define _PID_AUTOTUNE_H

#include <stdint.h>

#include "Module.h"

class TemperatureControl;

class PID_Autotuner : public Module
{
public:
    PID_Autotuner();

    void on_module_loaded(void);
    uint32_t on_tick(uint32_t);
    void on_idle(void *);
    void on_gcode_received(void *);

private:
    void begin(float, int );
    void abort();
    void finishUp();

    TemperatureControl *temp_control;
    float target_temperature;

    float *peaks;
    int requested_cycles;
    float noiseBand;
    unsigned long peak1, peak2;
    int sampleTime;
    int nLookBack;
    int lookBackCnt;
    int peakType;
    float *lastInputs;
    int peakCount;
    float absMax, absMin;
    float oStep;
    int output;
    volatile unsigned long tickCnt;
    struct {
        bool justchanged:1;
        volatile bool tick:1;
        bool firstPeak:1;
    };
};

#endif /* _PID_AUTOTUNE_H */
