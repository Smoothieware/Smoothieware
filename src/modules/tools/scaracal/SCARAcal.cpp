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
//#include "mbed.h"
//#include "IAP.h"

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

    // tracking calibration positions for manual calibration of bed level
    // TODO - remove this when probing sectoin done and working
    this->cal[0] = 0;
    this->cal[1] = 0;
    this->cal[2] = 10;       // Bed moves in to 10mm off the nozzle to prevent damage
    this->in_cal = false;

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

void SCARAcal::move(float *position, float feed)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", position[0], position[1], position[2], feed * 60); // use specified feedrate (mm/sec)

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
}

void SCARAcal::SCARA_ang_move(float theta, float psi, float z, float feedrate)
{
    float actuator[3],
          cartesian[3];

    // Assign the actuator angles from input
    actuator[0] = theta;
    actuator[1] = psi;
    actuator[2] = z;

    // Calculate the physical position relating to the arm angles
    THEKERNEL->robot->arm_solution->actuator_to_cartesian( actuator, cartesian );

    this->move(cartesian, feedrate);
}

void SCARAcal::next_cal(void){
    if (this->cal[X_AXIS] == 50 || this->cal[X_AXIS] == 150){
        this->cal[Y_AXIS] -= 50;
        if (this->cal[Y_AXIS] < 0){
            this->cal[Y_AXIS] = 0;
            this->cal[X_AXIS] += 50;
        }
    }
    else {
        this->cal[Y_AXIS] += 50;
        if (this->cal[Y_AXIS] > 200){
            this->cal[X_AXIS] += 50;
            if (this->cal[X_AXIS] > 200){
                this->cal[X_AXIS] = 0;
                this->cal[Y_AXIS] = 0;
            }
            else
                this->cal[Y_AXIS] = 200;
        }
    }
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
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate); // move to target
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

                    STEPPER[0]->change_steps_per_mm(actuators[0] / target[0] * STEPPER[0]->get_steps_per_mm()); // Find angle difference
                    STEPPER[1]->change_steps_per_mm(STEPPER[0]->get_steps_per_mm());  // and change steps_per_mm to ensure correct steps per *angle* 
                } else {
                    this->home();                                                   // home - This time leave trims as adjusted.
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate); // move to target
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

                    THEKERNEL->robot->get_axis_position(cartesian);                                     // get actual position from robot
                    THEKERNEL->robot->arm_solution->cartesian_to_actuator( cartesian, actuators );      // translate it to get actual actuator angles

                    S_delta[1] = actuators[1] - target[1];                 // Find difference, and 
                    set_trim(S_trim[0], S_delta[1], 0, gcode->stream);     // set trim to reflect the difference
                } else {
                    set_trim(S_trim[0], 0, 0, gcode->stream);               // reset trim for calibration move
                    this->home();                                                   // home
                    SCARA_ang_move(target[0], target[1], 100.0F, slow_rate); // move to target
                }
                gcode->mark_as_taken();
            }
            return;

            //Additional section for Morgan bed calbration: M370 - M375
            // M370: Clear current grid for calibration, and move to first position
            case 370: {
                this->home();
                for (int i=0; i<25; i++){
                    THEKERNEL->robot->bed_level_data.pData[i] = 0.0F;        // Clear the grid
                }

                this->cal[X_AXIS] = 0;                                              // Clear calibration position
                this->cal[Y_AXIS] = 0;
                this->in_cal = true;                                         // In calbration mode

            }
            return;
            // M372: save current position in grid, and move to next initial position
            case 372: {
                if (in_cal){
                    float cartesian[3];
                    int pindex = 0;

                    THEKERNEL->robot->get_axis_position(cartesian);         // get actual position from robot
                    pindex = (int) ((cartesian[X_AXIS]/10.0F)+(cartesian[Y_AXIS]/50.0F));

                    this->move(this->cal, slow_rate);                       // move to the next position
                    this->next_cal();                                       // to not cause damage to machine due to Z-offset

                    THEKERNEL->robot->bed_level_data.pData[pindex] = 0.0F + cartesian[Z_AXIS];  // save the offset

                }                   
            }
            return;
            // M371: Move to next initial position
            case 371: {
                if (in_cal){
                    this->move(this->cal, slow_rate);
                    this->next_cal();
                }

            }
            return;
            // M373: finalize calibration  
            case 373: {
                 this->in_cal = false;

            }
            return;
            // M374: manually inject calibration - Alphabetical grid assignment
            case 374: {
                int i=0,
                    x=0;
                
                if(gcode->has_letter('X')) { // Column nr (X)
                    x = gcode->get_value('X');
                }
                if (x<5){                    // Only for valid rows
                    for (i=0; i<5; i++){
                        if(gcode->has_letter('A'+i)) {
                            THEKERNEL->robot->bed_level_data.pData[i+(x*5)] = gcode->get_value('A'+i);
                        }
                    }
                }    
            }
            return;
            // M375: Display grid data in terminal
            case 375: {
                
            }
            return;
            case 500: // M500 saves some volatile settings to config override file
            case 503:  // M503 just prints the settings

                // Bed grid data as gcode: Still not working right...
                gcode->stream->printf(";Bed Level settings:\n");
                int x,y;
                for (x=0; x<5; x++){
                    gcode->stream->printf("M374 X%i",x);
                    for (y=0; y<5; y++){
                         gcode->stream->printf(" %c%1.2f", 'A'+y, THEKERNEL->robot->bed_level_data.pData[(x*5)+y]);
                    }
                    gcode->stream->printf("\n"); 
                }  
                gcode->mark_as_taken();
                break;
            
            return;

        }
    }    
}

