/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SCARAcal.h"

#include "Kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "Stepper.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SerialMessage.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include <algorithm>
#include <map>

#define scaracal_checksum CHECKSUM("scaracal")
#define enable_checksum CHECKSUM("enable")
#define slow_feedrate_checksum CHECKSUM("slow_feedrate")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())

void SCARAcal::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( scaracal_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
}

void SCARAcal::on_config_reload(void *argument)
{
    this->slow_rate = THEKERNEL->config->value( scaracal_checksum, slow_feedrate_checksum )->by_default(5)->as_number(); // feedrate in mm/sec

}


// issue home command
void SCARAcal::home()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

bool SCARAcal::get_trim(float& x, float& y, float& z)
{
    void *returned_data;
    bool ok = PublicData::get_value( endstops_checksum, trim_checksum, &returned_data );

    if (ok) {
        float *trim = static_cast<float *>(returned_data);
        x= trim[0];
        y= trim[1];
        z= trim[2];
        return true;
    }
    return false;
}

bool SCARAcal::set_trim(float x, float y, float z, StreamOutput *stream)
{
    float t[3]{x, y, z};
    bool ok= PublicData::set_value( endstops_checksum, trim_checksum, t);

    if (ok) {
        stream->printf("set trim to X:%f Y:%f Z:%f\n", x, y, z);
    } else {
        stream->printf("unable to set trim, is endstops enabled?\n");
    }

    return ok;
}

bool SCARAcal::get_home_offset(float& x, float& y, float& z)
{
    void *returned_data;
    bool ok = PublicData::get_value( endstops_checksum, home_offset_checksum, &returned_data );

    if (ok) {
        float *home_offset = static_cast<float *>(returned_data);
        x= home_offset[0];
        y= home_offset[1];
        z= home_offset[2];
        return true;
    }
    return false;
}

bool SCARAcal::set_home_offset(float x, float y, float z, StreamOutput *stream)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "M206 X%1.3f Y%1.3f Z%1.3f", x, y, z); // Send saved Z homing offset

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

    stream->printf("Set home_offset to X:%f Y:%f Z:%f\n", x, y, z);

    return true;//ok;
}

bool SCARAcal::translate_trim(StreamOutput *stream)
{
    float S_trim[3],
          home_off[3],
          actuator[3];;

    this->get_home_offset(home_off[0], home_off[1], home_off[2]);               // get home offsets
    this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	                          // get current trim

    THEKERNEL->robot->arm_solution->cartesian_to_actuator( home_off, actuator ); // convert current home offset to actuator angles

    // Subtract trim values from actuators to determine the real home offset actuator position for X and Y.

    actuator[0] -= S_trim[0];
    actuator[1] -= S_trim[1];

    // Clear X and Y trims internally
    S_trim[0] = 0.0F;
    S_trim[1] = 0.0F;

    // convert back to get the real cartesian home offsets

    THEKERNEL->robot->arm_solution->actuator_to_cartesian( actuator, home_off );

    this->set_home_offset(home_off[0], home_off[1], home_off[2],stream);               // get home offsets
                 // Set the correct home offsets;

    this->set_trim(S_trim[0], S_trim[1], S_trim[2], stream);    // Now Clear relevant trims

    return true;
}

void SCARAcal::SCARA_ang_move(float theta, float psi, float z, float feedrate)
{
    char cmd[64];
    float actuator[3],
          cartesian[3];

    // Assign the actuator angles from input
    actuator[0] = theta;
    actuator[1] = psi;
    actuator[2] = z;

    // Calculate the physical position relating to the arm angles
    THEKERNEL->robot->arm_solution->actuator_to_cartesian( actuator, cartesian );

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", cartesian[0], cartesian[1], cartesian[2], feedrate * 60); // use specified feedrate (mm/sec)

    //THEKERNEL->streams->printf("DEBUG: move: %s\n", cmd);

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
}

//A GCode has been received
//See if the current Gcode line has some orders for us
void SCARAcal::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        switch( gcode->m ) {

            case 114: {    // Extra stuff for Morgan calibration
                char buf[32];
                float cartesian[3],
                      actuators[3];

                THEKERNEL->robot->get_axis_position(cartesian);    // get actual position from robot
                THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate to get actuator position

                int n = snprintf(buf, sizeof(buf), "  A: Th:%1.3f Ps:%1.3f",
                                 actuators[0],
                                 actuators[1]);    // display actuator angles Theta and Psi.
                gcode->txt_after_ok.append(buf, n);
            }
            break;

            case 360: {
                float target[2] = {0.0F, 120.0F},
                      S_trim[3];

                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	// get current trim to conserve other calbration values

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    float cartesian[3],
                          actuators[3],
                          S_delta[2],
                          S_trim[3];

                    THEKERNEL->robot->get_axis_position(cartesian);    // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate to get actuator position

                    S_delta[0] = actuators[0] - target[0];

                    set_trim(S_delta[0], S_trim[1], 0, gcode->stream);
                } else {
                    set_trim(0, S_trim[1], 0, gcode->stream);               // reset trim for calibration move
                    this->home();                                                   // home
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate * 3.0F); // move to target
                }
            }
            break;

            case 361: {
                float target[2] = {90.0F, 130.0F};
                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    float cartesian[3],
                          actuators[3];

                    THEKERNEL->robot->get_axis_position(cartesian);                                // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators ); // translate to get actuator position

                    STEPPER[0]->change_steps_per_mm(actuators[0] / target[0] * STEPPER[0]->get_steps_per_mm()); // Find angle difference
                    STEPPER[1]->change_steps_per_mm(STEPPER[0]->get_steps_per_mm());  // and change steps_per_mm to ensure correct steps per *angle*
                } else {
                    this->home();                                                   // home - This time leave trims as adjusted.
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate * 3.0F); // move to target
                }

            }
            break;

            case 364: {
                float target[2] = {45.0F, 135.0F},
                      S_trim[3];

                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	// get current trim to conserve other calbration values

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    float cartesian[3],
                          actuators[3],
                          S_delta[2];

                    THEKERNEL->robot->get_axis_position(cartesian);                                     // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate it to get actual actuator angles

                    S_delta[1] = ( actuators[1] - actuators[0]) - ( target[1] - target[0] );            // Find difference in angle - not actuator difference, and
                    set_trim(S_trim[0], S_delta[1], 0, gcode->stream);                                  // set trim to reflect the difference
                } else {
                    set_trim(S_trim[0], 0, 0, gcode->stream);                                           // reset trim for calibration move
                    this->home();                                                                       // home
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate * 3.0F);                     // move to target
                }
            }
            break;

            /* TODO case 365: {   // set scara scaling
              std::map<char, float> buf = gcode->get_args();
              //algorithm::replace(buf.begin, buf.end , 'X', 'A');
              //algorithm::algorithm::replace(buf.begin, buf.end , 'Y', 'B');
              //algorithm::replace(buf.begin, buf.end , 'Z', 'C');

              buf = string("M665 ") + buf;

              Gcode gc(buf, &(StreamOutput::NullStream));
              THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

              //stream->printf("Set scaling to X:%f Y:%f Z:%f\n", x, y, z);

        */

            case 366:                                       // Translate trims to the actual endstop offsets for SCARA
                this->translate_trim(gcode->stream);
                break;

        }
    }
}

