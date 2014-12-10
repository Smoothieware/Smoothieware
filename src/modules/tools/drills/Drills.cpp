/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "Drills.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Robot.h"
#include "Conveyor.h"
#include "SlowTicker.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include <math.h> /* fmod */

// axis index
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// retract modes
#define RETRACT_TO_Z 0
#define RETRACT_TO_R 1

// dwell unity
#define DWELL_UNITY_S 0 // seconds
#define DWELL_UNITY_P 1 // millis

// config names
#define drills_checksum      CHECKSUM("drills")
#define enable_checksum      CHECKSUM("enable")
#define dwell_unity_checksum CHECKSUM("dwell_unity")

Drills::Drills() {}

void Drills::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(! THEKERNEL->config->value(drills_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    
    // Settings
    this->on_config_reload(this);

    // events
    this->register_for_event(ON_GCODE_RECEIVED);

    // reset values
    this->cycle_started = false;
    this->retract_type  = RETRACT_TO_Z;

    this->initial_z = 0;
    this->r_plane   = 0;

    this->reset_sticky();
}

void Drills::on_config_reload(void *argument)
{
    // take the dwell unity configured by user, or select S (seconds) by default
    string dwell_unity = THEKERNEL->config->value(drills_checksum, dwell_unity_checksum)->by_default("S")->as_string();
    this->dwell_unity  = (dwell_unity == "P") ? DWELL_UNITY_P : DWELL_UNITY_S;
}

/*
The canned cycles have been implemented as described on this page :
http://www.tormach.com/g81_g89_backgroung.html

/!\ This code expects a clean gcode, no fail safe at this time.

Implemented     : G80-83, G98, G99
Absolute mode   : yes
Relative mode   : no
Incremental (L) : no
*/

/* reset all sticky values, called before each cycle */
void Drills::reset_sticky()
{
    this->sticky_z = 0; // Z depth
    this->sticky_r = 0; // R plane
    this->sticky_f = 0; // feedrate
    this->sticky_q = 0; // peck drilling increment
    this->sticky_p = 0; // dwell in seconds
}

/* update all sticky values, called before each hole */
void Drills::update_sticky(Gcode *gcode)
{
    if (gcode->has_letter('Z')) this->sticky_z = gcode->get_value('Z');
    if (gcode->has_letter('R')) this->sticky_r = gcode->get_value('R');
    if (gcode->has_letter('F')) this->sticky_f = gcode->get_value('F');
    if (gcode->has_letter('Q')) this->sticky_q = gcode->get_value('Q');
    if (gcode->has_letter('P')) this->sticky_p = gcode->get_int('P');
    
    // set retract plane
    if (this->retract_type == RETRACT_TO_Z) 
        this->r_plane = this->initial_z;
    else 
        this->r_plane = this->sticky_r;
}

/* send a formatted Gcode line */
int Drills::send_gcode(const char* format, ...)
{
    // handle variable arguments
    va_list args;
    va_start(args, format);
    // make the formatted string
    char line[32]; // max length for an gcode line
    int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    // debug, print the gcode sended
    //THEKERNEL->streams->printf(">>> %s\r\n", line);
    // make gcode object and send it (right way)
    Gcode gc(line, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
    // return the gcode srting length
    return n;
}

/* G83: peck drilling */
void Drills::peck_hole()
{
    // start values
    float depth  = this->sticky_r - this->sticky_z; // travel depth
    float cycles = depth / this->sticky_q;          // cycles count
    float rest   = fmod(depth, this->sticky_q);     // final pass
    float z_pos  = this->sticky_r;                  // current z position

    // for each cycle
    for (int i = 1; i < cycles; i++) {
        // decrement depth
        z_pos -= this->sticky_q;
        // feed down to depth at feedrate (F and Z)
        this->send_gcode("G1 F%1.4f Z%1.4f", this->sticky_f, z_pos);
        // rapids to retract position (R)
        this->send_gcode("G0 Z%1.4f", this->sticky_r);
    }

    // final depth not reached
    if (rest > 0) {
        // feed down to final depth at feedrate (F and Z)
        this->send_gcode("G1 F%1.4f Z%1.4f", this->sticky_f, this->sticky_z);
    }
}

void Drills::make_hole(Gcode *gcode)
{
    // compile X and Y values
    char x[16] = "";
    char y[16] = "";
    if (gcode->has_letter('X'))
        snprintf(x, sizeof(x), " X%1.4f", gcode->get_value('X'));
    if (gcode->has_letter('Y'))
        snprintf(y, sizeof(y), " Y%1.4f", gcode->get_value('Y'));

    // rapids to X/Y
    this->send_gcode("G0%s%s", x, y);
    // rapids to retract position (R)
    this->send_gcode("G0 Z%1.4f", this->sticky_r);
    
    // if peck drilling
    if (this->sticky_q > 0) 
        this->peck_hole();
    else
        // feed down to depth at feedrate (F and Z)
        this->send_gcode("G1 F%1.4f Z%1.4f", this->sticky_f, this->sticky_z);
    
    // if dwell, wait for x seconds
    if (this->sticky_p > 0) {
        // dwell exprimed in seconds
        if (this->dwell_unity == DWELL_UNITY_S)
            this->send_gcode("G4 S%u", this->sticky_p);
        // dwell exprimed in milliseconds
        else
            this->send_gcode("G4 P%u", this->sticky_p);
    }
    
    // rapids retract at R-Plane (Initial-Z or R)
    this->send_gcode("G0 Z%1.4f", this->r_plane);
}

void Drills::on_gcode_received(void* argument)
{
    // received gcode
    Gcode *gcode = static_cast<Gcode *>(argument);

    // no "G" in gcode, exit...
    if (! gcode->has_g) 
        return;
    
    // "G" value
    int code = gcode->g;

    // cycle start
    if (code == 98 || code == 99) {
        // wait for any moves left and current position is update
        THEKERNEL->conveyor->wait_for_empty_queue();
        // get actual position from robot
        float pos[3];
        THEKERNEL->robot->get_axis_position(pos);
        // backup Z position as Initial-Z value
        this->initial_z = pos[Z_AXIS];
        // set retract type
        this->retract_type = (code == 98) ? RETRACT_TO_Z : RETRACT_TO_R;
        // reset sticky values
        this->reset_sticky();
        // mark cycle started and gcode taken
        this->cycle_started = true;
        gcode->mark_as_taken();
    }
    // cycle end
    else if (code == 80) {
        // mark cycle endded and gcode taken
        this->cycle_started = false;
        gcode->mark_as_taken();
        // if retract position is R-Plane
        if (this->retract_type == RETRACT_TO_R) {
            // rapids retract at Initial-Z to avoid futur collisions
            this->send_gcode("G0 Z%1.4f", this->initial_z);
        }
    }
    // in cycle
    else if (this->cycle_started) {
        // relative mode not supported for now...
        if (THEKERNEL->robot->absolute_mode == false) {
            gcode->stream->printf("Drills: relative mode not supported.\r\n");
            gcode->stream->printf("Drills: skip hole...\r\n");
            // exit
            return;
        }
        // implemented cycles
        if (code == 81 || code == 82 || code == 83) {
            this->update_sticky(gcode);
            this->make_hole(gcode);
            gcode->mark_as_taken();
        }
    }
}

