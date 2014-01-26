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

#define enable_checksum CHECKSUM("enable")

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

        endstop->pool = this;

        endstops.push_back( endstop );
        endstop->pool_index = endstops.size() - 1;

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
            switch(endstop->axis())
            {
                case 0: // X axis
                    if (endstop->dir())
                        THEKERNEL->robot->x_max = endstop;
                    else
                        THEKERNEL->robot->x_min = endstop;
                    break;
                case 1: // Y axis
                    if (endstop->dir())
                        THEKERNEL->robot->y_max = endstop;
                    else
                        THEKERNEL->robot->y_min = endstop;
                    break;
                case 2: // Z axis
                    if (endstop->dir())
                        THEKERNEL->robot->z_max = endstop;
                    else
                        THEKERNEL->robot->z_min = endstop;
                    break;
            }
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
                break;
            case 500: // M500 save settings
                break;
            case 503: // M503 print settings
                break;
            case 665: // M665 set max gamma/Z height
                break;
            case 666: // M666 set axis trim
                break;
            case 910: // M910 move specific # of steps
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
