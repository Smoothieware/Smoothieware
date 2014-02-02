/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Endstops.h"

#include "Kernel.h"
#include "Endstop.h"
#include "StepperMotor.h"

Endstops* Endstops::instance = NULL;

Endstops::Endstops()
{
    instance = this;
}

void Endstops::on_module_loaded()
{
    vector<uint16_t> names;

    THEKERNEL->config->get_module_list( &names, endstop_checksum );

    for (uint16_t name : names)
    {
        Endstop* endstop = new Endstop(name);

        endstop->pool_index = endstops.size();
        endstops.push_back( endstop );

        THEKERNEL->add_module(endstop);

        // fill appropriate pointers
        // TODO: make this based on matching names in config instead of being hyper-explicit
        //       or at least find a neater way to do it
        if (endstop->type == ENDSTOP_TYPE_ACTUATOR)
        {
            if (endstop->dir())
                THEKERNEL->robot->actuators[endstop->axis()]->max_stop = endstop;
            else
                THEKERNEL->robot->actuators[endstop->axis()]->min_stop = endstop;
        }
        else if (endstop->type == ENDSTOP_TYPE_CARTESIAN)
        {
            if (endstop->dir())
                THEKERNEL->robot->axes[endstop->axis()].max_stop = endstop;
            else
                THEKERNEL->robot->axes[endstop->axis()].min_stop = endstop;
        }
    }

    THEKERNEL->slow_ticker->attach( 1000, this, &Endstops::poll );

    register_for_event(ON_GCODE_RECEIVED);
}

void Endstops::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m)
    {
        switch(gcode->m)
        {
            case 119: // M119 report endstop status
                for (Endstop* e : endstops)
                    gcode->stream->printf("%s: %s ", e->name, e->asserted()?"TRG":"non");
                if (endstops.size())
                    gcode->stream->puts("\n");
                break;
            case 206: // M206 set homing offset
            {
                // TODO: allow user to choose max/min when both are specified in config
                // TODO: support arbitrary number of actuators
                for (int i = 0; i < 3; i++)
                {
                    if (gcode->has_letter('A' + i))
                    {
                        if (THEKERNEL->robot->actuators[i]->min_stop)
                            THEKERNEL->robot->actuators[i]->min_stop->position = gcode->get_value('X' + i);
                        else if (THEKERNEL->robot->actuators[i]->max_stop)
                            THEKERNEL->robot->actuators[i]->max_stop->position = gcode->get_value('X' + i);
                    }
                    if (gcode->has_letter('X' + i))
                    {
                        if (THEKERNEL->robot->axes[i].min_stop)
                            THEKERNEL->robot->axes[i].min_stop->position = gcode->get_value('X' + i);
                        else if (THEKERNEL->robot->axes[i].max_stop)
                            THEKERNEL->robot->axes[i].max_stop->position = gcode->get_value('X' + i);
                    }
                }
                break;
            }
            case 500: // M500 save settings
                // TODO
                break;
            case 503: // M503 print settings
                // TODO
                break;
            case 665: // M665 set max gamma/Z height
                // TODO
                break;
            case 666: // M666 set axis trim
                // TODO
                break;
        }
    }
}

uint32_t Endstops::poll(uint32_t)
{
    for (Endstop* e : endstops)
        e->poll();

    return 0;
}

Endstop* Endstops::find_endstop_by_name(uint16_t name)
{
    for (Endstop* e : endstops)
    {
        if (e->myname_checksum == name)
            return e;
    }
    return NULL;
}

bool Endstops::find_endstops_by_type(std::vector<Endstop*> & set, Endstop_Types_e type)
{
    for (Endstop* e : endstops)
    {
        if (e->type == type)
            set.push_back(e);
    }
    return (set.size() > 0);
}
