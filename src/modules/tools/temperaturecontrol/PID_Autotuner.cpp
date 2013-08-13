#include "PID_Autotuner.h"

#include "Kernel.h"

PID_Autotuner::PID_Autotuner()
{
    t = NULL;
    s = NULL;
}

void PID_Autotuner::on_module_loaded()
{
    tick = false;
    this->kernel->slow_ticker->attach(20, this, &PID_Autotuner::on_tick );
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_RECEIVED);
}

void PID_Autotuner::begin(TemperatureControl *temp, double target, StreamOutput *stream)
{
    if (t)
        t->heater_pin.set(0);

    s = stream;
    t = temp;

    t->target_temperature = 0.0;

    target_temperature = target;

    for (cycle = 0; cycle < 8; cycle++)
    {
        cycles[cycle].ticks_high = 0;
        cycles[cycle].ticks_low  = 0;
        cycles[cycle].t_max      = 0.0;
        cycles[cycle].t_min      = 1000.0;
    }
    cycle = 0;

    s->printf("%s: Starting PID Autotune, M304 aborts\n", t->designator.c_str());

    bias = d = t->heater_pin.max_pwm() >> 1;

    output = true;
    last_output = true;
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
}

void PID_Autotuner::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);

    if ((gcode->has_m) && (gcode->m == 304))
        abort();
}

uint32_t PID_Autotuner::on_tick(uint32_t dummy)
{
    if (t)
        tick = true;
    return 0;
}

void PID_Autotuner::on_idle(void*)
{
    if (!tick)
        return;

    tick = false;

    if (cycle >= PID_AUTOTUNER_CYCLES)
        return;
    if (t == NULL)
        return;

    if (t->last_reading > (target_temperature + 0.25))
        output = false;
    else if (t->last_reading < (target_temperature - 0.25))
        output = true;

    if (last_output == false && output)
    {
        s->printf("Cycle %d:\n\tMax: %5.1f  Min: %5.1f  high time: %3.1fs  low time: %3.1fs\n", cycle, cycles[cycle].t_max, cycles[cycle].t_min, cycles[cycle].ticks_high / 20.0, cycles[cycle].ticks_low / 20.0);

        // this code taken from http://github.com/ErikZalm/Marlin/blob/Marlin_v1/Marlin/temperature.cpp
        bias += (d * (cycles[cycle].ticks_high - cycles[cycle].ticks_low) * (1000.0 / 20.0)) / ((cycles[cycle].ticks_high + cycles[cycle].ticks_low) * (1000.0 / 20.0));
        bias = confine(bias, 20, t->heater_pin.max_pwm() - 20);
        if (bias > (t->heater_pin.max_pwm() / 2))
            d = t->heater_pin.max_pwm() - 1 - bias;
        else
            d = bias;
        // end code from Marlin firmware

        cycle++;
        if (cycle == PID_AUTOTUNER_CYCLES)
        {
            t->heater_pin.set(0);
            t->set_desired_temperature(0.0);
            // TODO: finish
            double tmax_avg   = 0.0,
                   tmin_avg   = 0.0,
                   t_high_avg = 0.0,
                   t_low_avg  = 0.0;
            for (cycle = PID_AUTOTUNER_CYCLES - 3; cycle < PID_AUTOTUNER_CYCLES; cycle++)
            {
                tmax_avg   += cycles[cycle].t_max;
                tmin_avg   += cycles[cycle].t_min;
                t_high_avg += cycles[cycle].ticks_high;
                t_low_avg  += cycles[cycle].ticks_low;
            }
            tmax_avg   /= (PID_AUTOTUNER_CYCLES - 1.0);
            tmin_avg   /= (PID_AUTOTUNER_CYCLES - 1.0);
            t_high_avg /= (PID_AUTOTUNER_CYCLES - 1.0);
            t_low_avg  /= (PID_AUTOTUNER_CYCLES - 1.0);

            s->printf("Averages over last %d cycles: Max: %5.1fc  Min: %5.1fc  high samples: %3.0f  low samples: %3.0f\n", 3, tmax_avg, tmin_avg, t_high_avg, t_low_avg);

            // this code taken from http://github.com/ErikZalm/Marlin/blob/Marlin_v1/Marlin/temperature.cpp
            double ku = (4.0 * d) / (3.141592653589 * (tmax_avg - tmin_avg) / 2.0);
            double tu = (t_low_avg + t_high_avg) * (1000.0 / 20.0) / 1000.0;

            s->printf("\tku: %g\n\ttu: %g\n", ku, tu);

            double kp = 0.6 * ku;
            double ki = 2 * kp / tu / 20.0;
            double kd = kp * tu / 8.0;

            s->printf("\tTrying:\n\tKp: %5.1f\n\tKi: %5.3f\n\tKd: %5.0f\n", kp, ki, kd);
            // end code from Marlin Firmware

            t->setPIDp(kp);
            t->setPIDi(ki);
            t->setPIDd(kd);

            s->printf("PID Autotune Complete! The settings above have been loaded into memory, but not written to your config file.\n");

            t = NULL;
            s = NULL;

            return;
        }
        s->printf("Cycle %d:\n\tbias: %4d d: %4d\n", cycle, bias, d);
    }

    int ticks;

    if (output)
    {
        ticks = ++cycles[cycle].ticks_high;
        t->heater_pin.pwm((t->o = ((bias + d) >> 1)));
    }
    else
    {
        ticks = ++cycles[cycle].ticks_low;
        t->heater_pin.set((t->o = 0));
    }

    if ((ticks % 16) == 0)
    {
        s->printf("%s: %5.1f/%5.1f @%d %d %d/8\n", t->designator.c_str(), t->last_reading, target_temperature, t->o, (output?1:0), cycle);
    }

    if (t->last_reading > cycles[cycle].t_max)
        cycles[cycle].t_max = t->last_reading;

    if (t->last_reading < cycles[cycle].t_min)
        cycles[cycle].t_min = t->last_reading;

    last_output = output;
}
