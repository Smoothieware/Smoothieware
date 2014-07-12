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

#define scara_checksum           CHECKSUM("scara")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

void SCARAcal::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( scara_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // load settings
    this->on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_IDLE);
}

void SCARAcal::on_config_reload(void *argument)
{
    this->slow_rate = THEKERNEL->config->value(slow_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec

}

void SCARAcal::on_idle(void *argument)
{
}


// issue home command
void SCARAcal::home()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
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

void SCARAcal::SCARA_ang_move(float theta, float psi, float z, float feedrate)
{
    char buf[32];
    char cmd[64];
    unsigned int n;
    float actuator[3],
          cartesian[3];

    // Assign the actuator angles from input
    actuator[0] = theta;
    actuator[1] = psi;
    actuator[2] = z;

    // Calculate the physical position relating to the arm angles
    THEKERNEL->robot->arm_solution->actuator_to_cartesian( actuator, cartesian );

    // Assemble Gcode to add onto the queue
    strcpy(cmd, "G0 ");

    n = snprintf(buf, sizeof(buf), " X%1.3f", cartesian[0]);
    strncat(cmd, buf, n);

    n = snprintf(buf, sizeof(buf), " Y%1.3f", cartesian[1]);
    strncat(cmd, buf, n);

    n = snprintf(buf, sizeof(buf), " Z%1.3f", cartesian[2]);
    strncat(cmd, buf, n);
    

    // use specified feedrate (mm/sec)
    n = snprintf(buf, sizeof(buf), " F%1.1f", feedrate * 60); // feed rate is converted to mm/min
    strncat(cmd, buf, n);

    THEKERNEL->streams->printf("DEBUG: move: %s\n", cmd);

    // send as a command line as may have multiple G codes in it
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_empty_queue();
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

                int n = snprintf(buf, sizeof(buf), "A: Th:%1.3f Ps:%1.3f Th-St:%1.3f",
                                 actuators[0],
                                 actuators[1],
                                 actuators[0]/90*STEPS_PER_MM(X_AXIS));    // determine steps per mm when theta arm 90deg.
                gcode->txt_after_ok.append(buf, n);
                gcode->mark_as_taken();

            }
            return;
            
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
                gcode->mark_as_taken();
            }
            return;
            case 361: {
                float target[2] = {90.0F, 130.0F};
                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    float cartesian[3],
                          actuators[3];

                    THEKERNEL->robot->get_axis_position(cartesian);                                // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators ); // translate to get actuator position

                    THEKERNEL->robot->actuators[0]->change_steps_per_mm(actuators[0] / target[0] * THEKERNEL->robot->actuators[0]->get_steps_per_mm());
                    THEKERNEL->robot->actuators[1]->change_steps_per_mm(THEKERNEL->robot->actuators[0]->get_steps_per_mm());
                } else {
                    this->home();                                                   // home - This time leave trims as adjusted.
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate * 3.0F); // move to target
                }
                gcode->mark_as_taken();
            }
            return;
              case 364: {
                float target[2] = {45.0F, 135.0F},
                      S_trim[3];

                this->get_trim(S_trim[0], S_trim[1], S_trim[2]);	// get current trim to conserve other calbration values

                if(gcode->has_letter('P')) {
                    // Program the current position as target
                    float cartesian[3],
                          actuators[3],
                          S_delta[2];

                    THEKERNEL->robot->get_axis_position(cartesian);    // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate to get actuator position

                    S_delta[1] = actuators[1] - target[1];

                    set_trim(S_trim[0], S_delta[1], 0, gcode->stream);
                } else {
                    set_trim(S_trim[0], 0, 0, gcode->stream);               // reset trim for calibration move
                    this->home();                                                   // home
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate * 3.0F); // move to target
                }
                gcode->mark_as_taken();
            }
            return;
        }
    }    
}

