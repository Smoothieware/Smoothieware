#include "PID_Autotuner.h"
#include "Kernel.h"
#include "SlowTicker.h"
#include "Gcode.h"
#include "TemperatureControl.h"
#include "libs/StreamOutput.h"

#include <cmath>        // std::abs

//#define DEBUG_PRINTF s->printf
#define DEBUG_PRINTF(...)

PID_Autotuner::PID_Autotuner()
{
    t = NULL;
    s = NULL;
    lastInputs = NULL;
    peaks = NULL;
    tick = false;
    tickCnt= 0;
}

void PID_Autotuner::on_module_loaded()
{
    tick = false;
    THEKERNEL->slow_ticker->attach(20, this, &PID_Autotuner::on_tick );
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_RECEIVED);
}

void PID_Autotuner::begin(TemperatureControl *temp, float target, StreamOutput *stream, int ncycles)
{
    noiseBand = 0.5;
    oStep = temp->heater_pin.max_pwm(); // use max pwm to cycle temp
    nLookBack = 5 * 20; // 5 seconds of lookback
    lookBackCnt= 0;
    tickCnt= 0;

    if (lastInputs != NULL) delete[] lastInputs;
    lastInputs = new float[nLookBack+1];
    t = temp;
    s = stream;

    t->heater_pin.set(0);
    t->target_temperature = 0.0;

    target_temperature = target;
    requested_cycles = ncycles;

    if (peaks != NULL) delete[] peaks;
    peaks = new float[ncycles];

    for (int i = 0; i < ncycles; i++) {
        peaks[i] = 0.0;
    }

    peakType = 0;
    peakCount = 0;
    justchanged = false;

    float refVal = t->get_temperature();
    absMax = refVal;
    absMin = refVal;
    output= oStep;
    t->heater_pin.pwm(oStep); // turn on to start heating

    s->printf("%s: Starting PID Autotune, %d max cycles, M304 aborts\n", t->designator.c_str(), ncycles);
}

void PID_Autotuner::abort()
{
    if (!t)
        return;

    t->target_temperature = 0;
    t->heater_pin.set(0);
    t = NULL;

    if (s)
        s->printf("PID Autotune Aborted\n");
    s = NULL;

    if (peaks != NULL)
        delete[] peaks;
    peaks = NULL;
    if (lastInputs != NULL)
        delete[] lastInputs;
    lastInputs = NULL;
}

void PID_Autotuner::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if ((gcode->has_m) && (gcode->m == 304))
        abort();
}

uint32_t PID_Autotuner::on_tick(uint32_t dummy)
{
    if (t)
        tick = true;
    tickCnt += 1000/20; // millisecond tick count
    return 0;
}

/**
 * this autopid is based on https://github.com/br3ttb/Arduino-PID-AutoTune-Library/blob/master/PID_AutoTune_v0/PID_AutoTune_v0.cpp
 */
void PID_Autotuner::on_idle(void *)
{
    if (!tick)
        return;

    tick = false;

    if (t == NULL)
        return;

    if(peakCount >= requested_cycles) {
        finishUp();
        return;
    }

    float refVal = t->get_temperature();

    if (refVal > absMax) absMax = refVal;
    if (refVal < absMin) absMin = refVal;

    // oscillate the output base on the input's relation to the setpoint
    if (refVal > target_temperature + noiseBand){
        output= 0;
        //t->heater_pin.pwm(output);
        t->heater_pin.set(0);
    } else if (refVal < target_temperature - noiseBand) {
        output= oStep;
        t->heater_pin.pwm(output);
    }

    bool isMax = true, isMin = true;

    // id peaks
    for (int i = nLookBack - 1; i >= 0; i--) {
        float val = lastInputs[i];
        if (isMax) isMax = refVal > val;
        if (isMin) isMin = refVal < val;
        lastInputs[i + 1] = lastInputs[i];
    }

    lastInputs[0] = refVal;

    if (lookBackCnt < nLookBack) {
        lookBackCnt++; // count number of times we have filled lastInputs
        //we don't want to trust the maxes or mins until the inputs array has been filled
        return;
    }

    if (isMax) {
        if (peakType == 0) peakType = 1;
        if (peakType == -1) {
            peakType = 1;
            justchanged = true;
            peak2 = peak1;
        }
        peak1 = tickCnt;
        peaks[peakCount] = refVal;

    } else if (isMin) {
        if (peakType == 0) peakType = -1;
        if (peakType == 1) {
            peakType = -1;
            peakCount++;
            justchanged = true;
        }

        if (peakCount < requested_cycles) peaks[peakCount] = refVal;
    }

    // we need to ignore the first cycle warming up from room temp

    if (justchanged && peakCount > 2) {
        if(peakCount == 3) { // reset min to new min
            absMin= refVal;
        }
        //we've transitioned. check if we can autotune based on the last peaks
        float avgSeparation = (std::abs(peaks[peakCount - 1] - peaks[peakCount - 2]) + std::abs(peaks[peakCount - 2] - peaks[peakCount - 3])) / 2;
        s->printf("Cycle %d: max: %g, min: %g, avg separation: %g\n", peakCount, absMax, absMin, avgSeparation);
        if (peakCount > 3 && avgSeparation < 0.05 * (absMax - absMin)) {
            DEBUG_PRINTF("Stabilized\n");
            finishUp();
            return;
        }
    }

    justchanged = false;

    if ((tickCnt % 1000) == 0) {
        s->printf("%s: %5.1f/%5.1f @%d %d/%d\n", t->designator.c_str(), t->get_temperature(), target_temperature, output, peakCount, requested_cycles);
        DEBUG_PRINTF("lookBackCnt= %d, peakCount= %d, absmax= %g, absmin= %g, peak1= %lu, peak2= %lu\n", lookBackCnt, peakCount, absMax, absMin, peak1, peak2);
    }
}


void PID_Autotuner::finishUp()
{
    //we can generate tuning parameters!
    float Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
    float Pu = (float)(peak1-peak2) / 1000;
    s->printf("\tKu: %g, Pu: %g\n", Ku, Pu);

    float kp = 0.6 * Ku;
    float ki = 1.2 * Ku / Pu;
    float kd = Ku * Pu * 0.075;

    s->printf("\tTrying:\n\tKp: %5.1f\n\tKi: %5.3f\n\tKd: %5.0f\n", kp, ki, kd);

    t->setPIDp(kp);
    t->setPIDi(ki);
    t->setPIDd(kd);

    s->printf("PID Autotune Complete! The settings above have been loaded into memory, but not written to your config file.\n");


    // and clean up
    t->target_temperature = 0;
    t->heater_pin.set(0);
    t = NULL;
    s = NULL;

    if (peaks != NULL)
        delete[] peaks;
    peaks = NULL;

    if (lastInputs != NULL)
        delete[] lastInputs;
    lastInputs = NULL;
}
