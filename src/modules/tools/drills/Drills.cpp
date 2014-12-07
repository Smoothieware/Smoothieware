/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "Drills.h"
#include "Gcode.h"
#include "Robot.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"

#define STEPPER THEKERNEL->robot->actuators

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define RETRACT_TO_Z 0
#define RETRACT_TO_R 1

Drills::Drills() {}

void Drills::on_module_loaded()
{
    // events
    this->register_for_event(ON_GCODE_RECEIVED);

    // reset values
    this->cycle_started = false;
    this->retract_type  = RETRACT_TO_Z;

    this->initial_z = 0;
    this->r_plane   = 0;

    this->sticky_z = 0;
    this->sticky_r = 0;
    this->sticky_f = 0;
}

/*
The canned cycles have been implemented as described on this page :
http://www.tormach.com/g81_g89_backgroung.html

/!\ This code expects a clean gcode, no fail safe at this time.

Implemented     : G98, G99, G81
Incremental (L) : no
Absolute mode   : yes
Relative mode   : no
*/

// not useful for now, but when come G82, G83, etc...
void Drills::update_sticky(Gcode *gcode) {
    if (gcode->has_letter('Z')) this->sticky_z = gcode->get_value('Z');
    if (gcode->has_letter('R')) this->sticky_r = gcode->get_value('R');
    if (gcode->has_letter('F')) this->sticky_f = gcode->get_value('F');
    
    if (this->retract_type == RETRACT_TO_Z) 
        this->r_plane = this->initial_z;
    else 
        this->r_plane = this->sticky_r;
}

/* send a formatted Gcode line */
int Drills::send_line(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char line[32]; // max length for an gcode line
    int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    // send gcode
    Gcode gc(line, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
    return n;
}

/* G81: simple cycle */
void Drills::simple_cycle(Gcode *gcode)
{
    char x[16] = "";
    char y[16] = "";

    if (gcode->has_letter('X'))
        snprintf(x, sizeof(x), " X%1.4f", gcode->get_value('X'));
    if (gcode->has_letter('Y'))
        snprintf(y, sizeof(y), " Y%1.4f", gcode->get_value('Y'));

    // rapids to X/Y
    this->send_line("G0%s%s", x, y);
    // rapids to retract position (R)
    this->send_line("G0 Z%1.4f", this->sticky_r);
    // feed down to depth at feedrate (F and Z)
    this->send_line("G1 F%1.4f Z%1.4f", this->sticky_f, this->sticky_z);
    // rapids retract at R-Plane (Initial-Z or R)
    this->send_line("G0 Z%1.4f", this->r_plane);
}

void Drills::on_gcode_received(void* argument)
{
    // gcode line
    Gcode *gcode = static_cast<Gcode *>(argument);

    // no G in Gcode, exit...
    if (! gcode->has_g) 
        return;
    
    int code = gcode->g;

    if (code == 98 || code == 99) {
        THEKERNEL->conveyor->wait_for_empty_queue();
        this->initial_z = STEPPER[Z_AXIS]->get_current_position();
        this->retract_type = (code == 98) ? RETRACT_TO_Z : RETRACT_TO_R;
        this->cycle_started = true;
        gcode->mark_as_taken();
    }
    else if (code == 80) {
        this->cycle_started = false;
        gcode->mark_as_taken();
        // rapids retract at Initial-Z
        if (this->retract_type == RETRACT_TO_R) {
            this->send_line("G0 Z%1.4f", this->initial_z);
        }
    }
    else if (this->cycle_started) {
        // relative mode not supported for now...
        if (THEKERNEL->robot->absolute_mode == false) {
            gcode->stream->printf("Drills: relative mode not supported.\r\n");
            gcode->stream->printf("Drills: skip hole...\r\n");
            return;
        }
        // simple cycle
        if (code == 81) {
            this->update_sticky(gcode);
            this->simple_cycle(gcode);
            gcode->mark_as_taken();
        }
    }
}