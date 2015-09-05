#include "PID_Autotuner.h"
#include "Kernel.h"
#include "SlowTicker.h"
#include "Gcode.h"
#include "TemperatureControl.h"
#include "libs/StreamOutput.h"
#include "TemperatureControlPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"

#include <cmath>        // std::abs

//#define DEBUG_PRINTF s->printf
#define DEBUG_PRINTF(...)

PID_Autotuner::PID_Autotuner()
{
    temp_control = NULL;
    s = NULL;
    lastInputs = NULL;
    peaks = NULL;
    tick = false;
    tickCnt = 0;
    nLookBack = 10 * 20; // 10 seconds of lookback (fixed 20ms tick period)
}

void PID_Autotuner::on_module_loaded()
{
    tick = false;
    THEKERNEL->slow_ticker->attach(20, this, &PID_Autotuner::on_tick );
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_RECEIVED);
}

void PID_Autotuner::begin(float target, StreamOutput *stream, int ncycles)
{
    noiseBand = 0.5;
    oStep = temp_control->heater_pin.max_pwm(); // use max pwm to cycle temp
    lookBackCnt = 0;
    tickCnt = 0;

    if (lastInputs != NULL) delete[] lastInputs;
    lastInputs = new float[nLookBack + 1];

    s = stream;

    temp_control->heater_pin.set(0);
    temp_control->target_temperature = 0.0;

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
    firstPeak= false;
    output= 0;

    s->printf("%s: Starting PID Autotune, %d max cycles, M304 aborts\n", temp_control->designator.c_str(), ncycles);
}

void PID_Autotuner::abort()
{
    if (temp_control == NULL)
        return;

    temp_control->target_temperature = 0;
    temp_control->heater_pin.set(0);
    temp_control = NULL;

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

    if(gcode->has_m) {
        if(gcode->m == 304) {
            abort();

        } else if (gcode->m == 303 && gcode->has_letter('E')) {
            int pool_index = gcode->get_value('E');

            // get the temperature control instance with this pool index
            void *returned_data;
            bool ok = PublicData::get_value( temperature_control_checksum, pool_index_checksum, pool_index, &returned_data );

            if (ok) {
                this->temp_control =  *static_cast<TemperatureControl **>(returned_data);

            } else {
                gcode->stream->printf("No temperature control with index %d found\r\n", pool_index);
                return;
            }

            // set target
            float target = 150.0;
            if (gcode->has_letter('S')) {
                target = gcode->get_value('S');
                gcode->stream->printf("Target: %5.1f\n", target);
            }

            // set the cycles, really not needed for this new version
            int ncycles = 8;
            if (gcode->has_letter('C')) {
                ncycles = gcode->get_value('C');
                if(ncycles < 8) ncycles= 8;
            }

            // optionally set the noise band, default is 0.5
            if (gcode->has_letter('B')) {
                noiseBand = gcode->get_value('B');
            }

            // optionally set the look back in seconds default is 10 seconds
            if (gcode->has_letter('L')) {
                nLookBack = gcode->get_value('L');
            }

            gcode->stream->printf("Start PID tune for index E%d, designator: %s\n", pool_index, this->temp_control->designator.c_str());
            this->begin(target, gcode->stream, ncycles);
        }
    }
}

uint32_t PID_Autotuner::on_tick(uint32_t dummy)
{
    if (temp_control != NULL)
        tick = true;

    tickCnt += (1000 / 20); // millisecond tick count
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

    if (temp_control == NULL)
        return;

    if(peakCount >= requested_cycles) {
        s->printf("// WARNING: Autopid did not resolve within %d cycles, these results are probably innacurate\n", requested_cycles);
        finishUp();
        return;
    }

    float refVal = temp_control->get_temperature();

    // oscillate the output base on the input's relation to the setpoint
    if (refVal > target_temperature + noiseBand) {
        output = 0;
        //temp_control->heater_pin.pwm(output);
        temp_control->heater_pin.set(0);
        if(!firstPeak) {
            firstPeak= true;
            absMax= refVal;
            absMin= refVal;
        }

    } else if (refVal < target_temperature - noiseBand) {
        output = oStep;
        temp_control->heater_pin.pwm(output);
    }

    if ((tickCnt % 1000) == 0) {
        s->printf("// Autopid Status - %5.1f/%5.1f @%d %d/%d\n",  refVal, target_temperature, output, peakCount, requested_cycles);
    }

    if(!firstPeak){
        // we wait until we hit the first peak befire we do anything else,we need to ignore the itial warmup temperatures
        return;
    }

    // find the peaks high and low
    bool isMax = true, isMin = true;
    for (int i = nLookBack - 1; i >= 0; i--) {
        float val = lastInputs[i];
        if (isMax) isMax = refVal > val;
        if (isMin) isMin = refVal < val;
        lastInputs[i + 1] = lastInputs[i];
    }

    lastInputs[0] = refVal;

    //we don't want to trust the maxes or mins until the inputs array has been filled
    if (lookBackCnt < nLookBack) {
        lookBackCnt++; // count number of times we have filled lastInputs
        return;
    }

    if (isMax) {
        if(refVal > absMax) absMax= refVal;

        if (peakType == 0) peakType = 1;
        if (peakType == -1) {
            peakType = 1;
            justchanged = true;
            peak2 = peak1;
        }
        peak1 = tickCnt;
        peaks[peakCount] = refVal;

    } else if (isMin) {
        if(refVal < absMin) absMin= refVal;
        if (peakType == 0) peakType = -1;
        if (peakType == 1) {
            peakType = -1;
            peakCount++;
            justchanged = true;
        }

        if (peakCount < requested_cycles) peaks[peakCount] = refVal;
    }

    if (justchanged && peakCount >= 4) {
        // we've transitioned. check if we can autotune based on the last peaks
        float avgSeparation = (std::abs(peaks[peakCount - 1] - peaks[peakCount - 2]) + std::abs(peaks[peakCount - 2] - peaks[peakCount - 3])) / 2;
        s->printf("// Cycle %d: max: %g, min: %g, avg separation: %g\n", peakCount, absMax, absMin, avgSeparation);
        if (peakCount > 3 && avgSeparation < (0.05 * (absMax - absMin))) {
            DEBUG_PRINTF("Stabilized\n");
            finishUp();
            return;
        }
    }

    if ((tickCnt % 1000) == 0) {
        DEBUG_PRINTF("lookBackCnt= %d, peakCount= %d, absmax= %g, absmin= %g, peak1= %lu, peak2= %lu\n", lookBackCnt, peakCount, absMax, absMin, peak1, peak2);
    }

    justchanged = false;
}


void PID_Autotuner::finishUp()
{
    //we can generate tuning parameters!
    float Ku = 4 * (2 * oStep) / ((absMax - absMin) * 3.14159);
    float Pu = (float)(peak1 - peak2) / 1000;
    s->printf("\tKu: %g, Pu: %g\n", Ku, Pu);

    float kp = 0.6 * Ku;
    float ki = 1.2 * Ku / Pu;
    float kd = Ku * Pu * 0.075;

    s->printf("\tTrying:\n\tKp: %5.1f\n\tKi: %5.3f\n\tKd: %5.0f\n", kp, ki, kd);

    temp_control->setPIDp(kp);
    temp_control->setPIDi(ki);
    temp_control->setPIDd(kd);

    s->printf("PID Autotune Complete! The settings above have been loaded into memory, but not written to your config file.\n");


    // and clean up
    temp_control->target_temperature = 0;
    temp_control->heater_pin.set(0);
    temp_control = NULL;
    s = NULL;

    if (peaks != NULL)
        delete[] peaks;
    peaks = NULL;

    if (lastInputs != NULL)
        delete[] lastInputs;
    lastInputs = NULL;
}
