#include "PID_Autotuner.h"

#include "Kernel.h"

PID_Autotuner::PID_Autotuner()
{
    t = NULL;
    s = NULL;
}

void PID_Autotuner::on_module_loaded()
{
    this->kernel->slow_ticker->attach(20, this, &PID_Autotuner::on_tick );
}

void PID_Autotuner::begin(TemperatureControl *temp, double target, StreamOutput *stream)
{
    if (t)
    {
        t->target_temperature = 0.0;
    }

    t = temp;

    t->p_factor = 1000000000.0;
    t->i_factor = 0.0;
    t->d_factor = 0.0;
    t->i_max    = 0.0;

    t->target_temperature = target;

    for (cycle = 0; cycle < 8; cycle++)
    {
        cycles[cycle].ticks = 0;
        cycles[cycle].t_max = 0.0;
        cycles[cycle].t_min = 1000.0;
    }
    cycle = 0;

    s = stream;

    s->printf("%s: Starting PID Autotune\n", t->designator.c_str());

    last_output = true;
}

uint32_t PID_Autotuner::on_tick(uint32_t dummy)
{
    if (cycle >= PID_AUTOTUNER_CYCLES)
        return 0;
    if (t == NULL)
        return 0;

    if (last_output == false && (t->o > 128))
    {
        s->printf("Cycle %d:\n\tMax: %5.1f  Min: %5.1f  time: %3.1fs\n", cycle, cycles[cycle].t_max, cycles[cycle].t_min, cycles[cycle].ticks / 20.0);
        cycle++;
        if (cycle == PID_AUTOTUNER_CYCLES)
        {
            t->set_desired_temperature(0.0);
            // TODO: finish
            double tmax_avg   = 0.0,
                   tmin_avg   = 0.0,
                   tcycle_avg = 0.0;
            for (cycle = 1; cycle < PID_AUTOTUNER_CYCLES; cycle++)
            {
                tmax_avg += cycles[cycle].t_max;
                tmin_avg += cycles[cycle].t_min;
                tcycle_avg += cycles[cycle].ticks;
            }
            tmax_avg /= 7.0;
            tmin_avg /= 7.0;
            tcycle_avg /= 7.0;
            s->printf("Averages over last %d cycles: Max: %5.1fc  Min: %5.1fc  samples: %3.0f\n", PID_AUTOTUNER_CYCLES - 2, tmax_avg, tmin_avg, tcycle_avg);

            // from http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
            // TODO: work out why the results of this algorithm are dreadfully poor
            double ku = 4 * 128 / ((tmax_avg - tmin_avg) * 3.141592653589);
            double pu = tcycle_avg;

            double kp = 0.6 * ku;
            double ki = 1.2 * ku / pu;
            double kd = 0.075 * ku * pu;

            s->printf("PID Autotune complete. Try M301 S%d P%4g I%4g D%4g\n", t->pool_index, kp, ki, kd);

            t->p_factor = kp;
            t->i_factor = ki;
            t->d_factor = kd;
            t->i_max = 128;

            t = NULL;
            s = NULL;

            return 0;
        }
    }

    last_output = (t->o > 128);

    cycles[cycle].ticks++;

    if ((cycles[cycle].ticks % 16) == 0)
    {
        s->printf("%s: %5.1f/%5.1f @%d %d %d/8\n", t->designator.c_str(), t->last_reading, t->target_temperature, t->o, (last_output?1:0), cycle);
    }

    if (t->last_reading > cycles[cycle].t_max)
        cycles[cycle].t_max = t->last_reading;

    if (t->last_reading < cycles[cycle].t_min)
        cycles[cycle].t_min = t->last_reading;

    return 0;
}
