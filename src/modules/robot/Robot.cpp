/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

#include <math.h>
#include <string>
using std::string;

#include "Planner.h"
#include "Conveyor.h"
#include "Robot.h"
#include "nuts_bolts.h"
#include "Pin.h"
#include "StepperMotor.h"
#include "Gcode.h"
#include "PublicDataRequest.h"
#include "RobotPublicAccess.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"
#include "arm_solutions/RotatableCartesianSolution.h"
#include "arm_solutions/RostockSolution.h"
#include "arm_solutions/JohannKosselSolution.h"
#include "arm_solutions/HBotSolution.h"
#include "StepTicker.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"

#define  default_seek_rate_checksum          CHECKSUM("default_seek_rate")
#define  default_feed_rate_checksum          CHECKSUM("default_feed_rate")
#define  mm_per_line_segment_checksum        CHECKSUM("mm_per_line_segment")
#define  delta_segments_per_second_checksum  CHECKSUM("delta_segments_per_second")
#define  mm_per_arc_segment_checksum         CHECKSUM("mm_per_arc_segment")
#define  arc_correction_checksum             CHECKSUM("arc_correction")
#define  x_axis_max_speed_checksum           CHECKSUM("x_axis_max_speed")
#define  y_axis_max_speed_checksum           CHECKSUM("y_axis_max_speed")
#define  z_axis_max_speed_checksum           CHECKSUM("z_axis_max_speed")

// arm solutions
#define  arm_solution_checksum               CHECKSUM("arm_solution")
#define  cartesian_checksum                  CHECKSUM("cartesian")
#define  rotatable_cartesian_checksum        CHECKSUM("rotatable_cartesian")
#define  rostock_checksum                    CHECKSUM("rostock")
#define  delta_checksum                      CHECKSUM("delta")
#define  hbot_checksum                       CHECKSUM("hbot")
#define  corexy_checksum                     CHECKSUM("corexy")
#define  kossel_checksum                     CHECKSUM("kossel")

// stepper motor stuff
#define  alpha_step_pin_checksum             CHECKSUM("alpha_step_pin")
#define  beta_step_pin_checksum              CHECKSUM("beta_step_pin")
#define  gamma_step_pin_checksum             CHECKSUM("gamma_step_pin")
#define  alpha_dir_pin_checksum              CHECKSUM("alpha_dir_pin")
#define  beta_dir_pin_checksum               CHECKSUM("beta_dir_pin")
#define  gamma_dir_pin_checksum              CHECKSUM("gamma_dir_pin")
#define  alpha_en_pin_checksum               CHECKSUM("alpha_en_pin")
#define  beta_en_pin_checksum                CHECKSUM("beta_en_pin")
#define  gamma_en_pin_checksum               CHECKSUM("gamma_en_pin")

#define  alpha_steps_per_mm_checksum         CHECKSUM("alpha_steps_per_mm")
#define  beta_steps_per_mm_checksum          CHECKSUM("beta_steps_per_mm")
#define  gamma_steps_per_mm_checksum         CHECKSUM("gamma_steps_per_mm")

#define  alpha_max_rate_checksum             CHECKSUM("alpha_max_rate")
#define  beta_max_rate_checksum              CHECKSUM("beta_max_rate")
#define  gamma_max_rate_checksum             CHECKSUM("gamma_max_rate")


// new-style actuator stuff
#define  actuator_checksum                   CHEKCSUM("actuator")

#define  step_pin_checksum                   CHECKSUM("step_pin")
#define  dir_pin_checksum                    CHEKCSUM("dir_pin")
#define  en_pin_checksum                     CHECKSUM("en_pin")

#define  steps_per_mm_checksum               CHECKSUM("steps_per_mm")
#define  max_rate_checksum                   CHECKSUM("max_rate")

#define  alpha_checksum                      CHECKSUM("alpha")
#define  beta_checksum                       CHECKSUM("beta")
#define  gamma_checksum                      CHECKSUM("gamma")


// The Robot converts GCodes into actual movements, and then adds them to the Planner, which passes them to the Conveyor so they can be added to the queue
// It takes care of cutting arcs into segments, same thing for line that are too long
#define max(a,b) (((a) > (b)) ? (a) : (b))

Robot::Robot(){
    this->inch_mode = false;
    this->absolute_mode = true;
    this->motion_mode =  MOTION_MODE_SEEK;
    this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    clear_vector(this->last_milestone);
    this->arm_solution = NULL;
    seconds_per_minute = 60.0F;
}

//Called when the module has just been loaded
void Robot::on_module_loaded() {
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Configuration
    this->on_config_reload(this);
}

void Robot::on_config_reload(void* argument){

    // Arm solutions are used to convert positions in millimeters into position in steps for each stepper motor.
    // While for a cartesian arm solution, this is a simple multiplication, in other, less simple cases, there is some serious math to be done.
    // To make adding those solution easier, they have their own, separate object.
    // Here we read the config to find out which arm solution to use
    if (this->arm_solution) delete this->arm_solution;
    int solution_checksum = get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());
    // Note checksums are not const expressions when in debug mode, so don't use switch
    if(solution_checksum == hbot_checksum || solution_checksum == corexy_checksum) {
        this->arm_solution = new HBotSolution(THEKERNEL->config);

    }else if(solution_checksum == rostock_checksum) {
        this->arm_solution = new RostockSolution(THEKERNEL->config);

    }else if(solution_checksum == kossel_checksum) {
        this->arm_solution = new JohannKosselSolution(THEKERNEL->config);

    }else if(solution_checksum ==  delta_checksum) {
        // place holder for now
        this->arm_solution = new RostockSolution(THEKERNEL->config);

    }else if(solution_checksum == rotatable_cartesian_checksum) {
        this->arm_solution = new RotatableCartesianSolution(THEKERNEL->config);

    }else if(solution_checksum == cartesian_checksum) {
        this->arm_solution = new CartesianSolution(THEKERNEL->config);

    }else{
        this->arm_solution = new CartesianSolution(THEKERNEL->config);
    }


    this->feed_rate           = THEKERNEL->config->value(default_feed_rate_checksum   )->by_default(  100.0F)->as_number();
    this->seek_rate           = THEKERNEL->config->value(default_seek_rate_checksum   )->by_default(  100.0F)->as_number();
    this->mm_per_line_segment = THEKERNEL->config->value(mm_per_line_segment_checksum )->by_default(    0.0F)->as_number();
    this->delta_segments_per_second = THEKERNEL->config->value(delta_segments_per_second_checksum )->by_default(0.0f   )->as_number();
    this->mm_per_arc_segment  = THEKERNEL->config->value(mm_per_arc_segment_checksum  )->by_default(    0.5f)->as_number();
    this->arc_correction      = THEKERNEL->config->value(arc_correction_checksum      )->by_default(    5   )->as_number();

    this->max_speeds[X_AXIS]  = THEKERNEL->config->value(x_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Y_AXIS]  = THEKERNEL->config->value(y_axis_max_speed_checksum    )->by_default(60000.0F)->as_number() / 60.0F;
    this->max_speeds[Z_AXIS]  = THEKERNEL->config->value(z_axis_max_speed_checksum    )->by_default(  300.0F)->as_number() / 60.0F;

    Pin alpha_step_pin;
    Pin alpha_dir_pin;
    Pin alpha_en_pin;
    Pin beta_step_pin;
    Pin beta_dir_pin;
    Pin beta_en_pin;
    Pin gamma_step_pin;
    Pin gamma_dir_pin;
    Pin gamma_en_pin;

    alpha_step_pin.from_string( THEKERNEL->config->value(alpha_step_pin_checksum )->by_default("2.0"  )->as_string())->as_output();
    alpha_dir_pin.from_string(  THEKERNEL->config->value(alpha_dir_pin_checksum  )->by_default("0.5"  )->as_string())->as_output();
    alpha_en_pin.from_string(   THEKERNEL->config->value(alpha_en_pin_checksum   )->by_default("0.4"  )->as_string())->as_output();
    beta_step_pin.from_string(  THEKERNEL->config->value(beta_step_pin_checksum  )->by_default("2.1"  )->as_string())->as_output();
    beta_dir_pin.from_string(   THEKERNEL->config->value(beta_dir_pin_checksum   )->by_default("0.11" )->as_string())->as_output();
    beta_en_pin.from_string(    THEKERNEL->config->value(beta_en_pin_checksum    )->by_default("0.10" )->as_string())->as_output();
    gamma_step_pin.from_string( THEKERNEL->config->value(gamma_step_pin_checksum )->by_default("2.2"  )->as_string())->as_output();
    gamma_dir_pin.from_string(  THEKERNEL->config->value(gamma_dir_pin_checksum  )->by_default("0.20" )->as_string())->as_output();
    gamma_en_pin.from_string(   THEKERNEL->config->value(gamma_en_pin_checksum   )->by_default("0.19" )->as_string())->as_output();

    float steps_per_mm[3] = {
        THEKERNEL->config->value(alpha_steps_per_mm_checksum)->by_default(  80.0F)->as_number(),
        THEKERNEL->config->value(beta_steps_per_mm_checksum )->by_default(  80.0F)->as_number(),
        THEKERNEL->config->value(gamma_steps_per_mm_checksum)->by_default(2560.0F)->as_number(),
    };

    // TODO: delete or detect old steppermotors
    // Make our 3 StepperMotors
    this->alpha_stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(alpha_step_pin, alpha_dir_pin, alpha_en_pin) );
    this->beta_stepper_motor   = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(beta_step_pin,  beta_dir_pin,  beta_en_pin ) );
    this->gamma_stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(gamma_step_pin, gamma_dir_pin, gamma_en_pin) );

    alpha_stepper_motor->change_steps_per_mm(steps_per_mm[0]);
    beta_stepper_motor->change_steps_per_mm(steps_per_mm[1]);
    gamma_stepper_motor->change_steps_per_mm(steps_per_mm[2]);

    alpha_stepper_motor->max_rate = THEKERNEL->config->value(alpha_max_rate_checksum)->by_default(30000.0F)->as_number() / 60.0F;
    beta_stepper_motor->max_rate  = THEKERNEL->config->value(beta_max_rate_checksum )->by_default(30000.0F)->as_number() / 60.0F;
    gamma_stepper_motor->max_rate = THEKERNEL->config->value(gamma_max_rate_checksum)->by_default(30000.0F)->as_number() / 60.0F;

    actuators.clear();
    actuators.push_back(alpha_stepper_motor);
    actuators.push_back(beta_stepper_motor);
    actuators.push_back(gamma_stepper_motor);

    // initialise actuator positions to current cartesian position (X0 Y0 Z0)
    // so the first move can be correct if homing is not performed
    float actuator_pos[3];
    arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);
    for (int i = 0; i < 3; i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);
}

void Robot::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        static float return_data;
        return_data = 100.0F * 60.0F / seconds_per_minute;
        pdr->set_data_ptr(&return_data);
        pdr->set_taken();

    }else if(pdr->second_element_is(current_position_checksum)) {
        static float return_data[3];
        return_data[0]= from_millimeters(this->last_milestone[0]);
        return_data[1]= from_millimeters(this->last_milestone[1]);
        return_data[2]= from_millimeters(this->last_milestone[2]);

        pdr->set_data_ptr(&return_data);
        pdr->set_taken();
    }
}

void Robot::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        // NOTE do not use this while printing!
        float t= *static_cast<float*>(pdr->get_data_ptr());
        // enforce minimum 10% speed
        if (t < 10.0F) t= 10.0F;

        this->seconds_per_minute = t / 0.6F; // t * 60 / 100
        pdr->set_taken();
    }
}

//A GCode has been received
//See if the current Gcode line has some orders for us
void Robot::on_gcode_received(void * argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    //Temp variables, constant properties are stored in the object
    uint8_t next_action = NEXT_ACTION_DEFAULT;
    this->motion_mode = -1;

   //G-letter Gcodes are mostly what the Robot module is interrested in, other modules also catch the gcode event and do stuff accordingly
    if( gcode->has_g){
        switch( gcode->g ){
            case 0:  this->motion_mode = MOTION_MODE_SEEK; gcode->mark_as_taken(); break;
            case 1:  this->motion_mode = MOTION_MODE_LINEAR; gcode->mark_as_taken();  break;
            case 2:  this->motion_mode = MOTION_MODE_CW_ARC; gcode->mark_as_taken();  break;
            case 3:  this->motion_mode = MOTION_MODE_CCW_ARC; gcode->mark_as_taken();  break;
            case 17: this->select_plane(X_AXIS, Y_AXIS, Z_AXIS); gcode->mark_as_taken();  break;
            case 18: this->select_plane(X_AXIS, Z_AXIS, Y_AXIS); gcode->mark_as_taken();  break;
            case 19: this->select_plane(Y_AXIS, Z_AXIS, X_AXIS); gcode->mark_as_taken();  break;
            case 20: this->inch_mode = true; gcode->mark_as_taken();  break;
            case 21: this->inch_mode = false; gcode->mark_as_taken();  break;
            case 90: this->absolute_mode = true; gcode->mark_as_taken();  break;
            case 91: this->absolute_mode = false; gcode->mark_as_taken();  break;
            case 92: {
                if(gcode->get_num_args() == 0){
                    clear_vector(this->last_milestone);
                }else{
                    for (char letter = 'X'; letter <= 'Z'; letter++){
                        if ( gcode->has_letter(letter) )
                            this->last_milestone[letter-'X'] = this->to_millimeters(gcode->get_value(letter));
                    }
                }

                // TODO: handle any number of actuators
                float actuator_pos[3];
                arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);

                for (int i = 0; i < 3; i++)
                    actuators[i]->change_last_milestone(actuator_pos[i]);

                gcode->mark_as_taken();
                return;
           }
       }
   }else if( gcode->has_m){
        switch( gcode->m ){
            case 92: // M92 - set steps per mm
                if (gcode->has_letter('X'))
                    actuators[0]->change_steps_per_mm(this->to_millimeters(gcode->get_value('X')));
                if (gcode->has_letter('Y'))
                    actuators[1]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Y')));
                if (gcode->has_letter('Z'))
                    actuators[2]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Z')));
                if (gcode->has_letter('F'))
                    seconds_per_minute = gcode->get_value('F');

                gcode->stream->printf("X:%g Y:%g Z:%g F:%g ", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm, seconds_per_minute);
                gcode->add_nl = true;
                gcode->mark_as_taken();
                return;
            case 114:
                {
                    char buf[32];
                    int n= snprintf(buf, sizeof(buf), "C: X:%1.3f Y:%1.3f Z:%1.3f",
                                                from_millimeters(this->last_milestone[0]),
                                                from_millimeters(this->last_milestone[1]),
                                                from_millimeters(this->last_milestone[2]));
                    gcode->txt_after_ok.append(buf, n);
                    gcode->mark_as_taken();
                }
                return;

            case 203: // M203 Set maximum feedrates in mm/sec
                if (gcode->has_letter('X'))
                    this->max_speeds[X_AXIS]= gcode->get_value('X');
                if (gcode->has_letter('Y'))
                    this->max_speeds[Y_AXIS]= gcode->get_value('Y');
                if (gcode->has_letter('Z'))
                    this->max_speeds[Z_AXIS]= gcode->get_value('Z');
                if (gcode->has_letter('A'))
                    alpha_stepper_motor->max_rate= gcode->get_value('A');
                if (gcode->has_letter('B'))
                    beta_stepper_motor->max_rate= gcode->get_value('B');
                if (gcode->has_letter('C'))
                    gamma_stepper_motor->max_rate= gcode->get_value('C');

                gcode->stream->printf("X:%g Y:%g Z:%g  A:%g B:%g C:%g ",
                    this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS],
                    alpha_stepper_motor->max_rate, beta_stepper_motor->max_rate, gamma_stepper_motor->max_rate);
                gcode->add_nl = true;
                gcode->mark_as_taken();
                break;

            case 204: // M204 Snnn - set acceleration to nnn, NB only Snnn is currently supported
                gcode->mark_as_taken();

                if (gcode->has_letter('S'))
                {
                    // TODO for safety so it applies only to following gcodes, maybe a better way to do this?
                    THEKERNEL->conveyor->wait_for_empty_queue();
                    float acc= gcode->get_value('S'); // mm/s^2
                    // enforce minimum
                    if (acc < 1.0F)
                        acc = 1.0F;
                    THEKERNEL->planner->acceleration= acc;
                }
                break;

            case 205: // M205 Xnnn - set junction deviation Snnn - Set minimum planner speed
                gcode->mark_as_taken();
                if (gcode->has_letter('X'))
                {
                    float jd= gcode->get_value('X');
                    // enforce minimum
                    if (jd < 0.0F)
                        jd = 0.0F;
                    THEKERNEL->planner->junction_deviation= jd;
                }
                if (gcode->has_letter('S'))
                {
                    float mps= gcode->get_value('S');
                    // enforce minimum
                    if (mps < 0.0F)
                        mps = 0.0F;
                    THEKERNEL->planner->minimum_planner_speed= mps;
                }
                break;

            case 220: // M220 - speed override percentage
                gcode->mark_as_taken();
                if (gcode->has_letter('S'))
                {
                    float factor = gcode->get_value('S');
                    // enforce minimum 10% speed
                    if (factor < 10.0F)
                        factor = 10.0F;
                    // enforce maximum 10x speed
                    if (factor > 1000.0F)
                        factor = 1000.0F;

                    seconds_per_minute = 6000.0F / factor;
                }
                break;

            case 400: // wait until all moves are done up to this point
                gcode->mark_as_taken();
                THEKERNEL->conveyor->wait_for_empty_queue();
                break;

            case 500: // M500 saves some volatile settings to config override file
            case 503: { // M503 just prints the settings
                gcode->stream->printf(";Steps per unit:\nM92 X%1.5f Y%1.5f Z%1.5f\n", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm);
                gcode->stream->printf(";Acceleration mm/sec^2:\nM204 S%1.5f\n", THEKERNEL->planner->acceleration);
                gcode->stream->printf(";X- Junction Deviation, S - Minimum Planner speed:\nM205 X%1.5f S%1.5f\n", THEKERNEL->planner->junction_deviation, THEKERNEL->planner->minimum_planner_speed);
                gcode->stream->printf(";Max feedrates in mm/sec, XYZ cartesian, ABC actuator:\nM203 X%1.5f Y%1.5f Z%1.5f A%1.5f B%1.5f C%1.5f\n",
                    this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS],
                    alpha_stepper_motor->max_rate, beta_stepper_motor->max_rate, gamma_stepper_motor->max_rate);

                // get or save any arm solution specific optional values
                BaseSolution::arm_options_t options;
                if(arm_solution->get_optional(options) && !options.empty()) {
                    gcode->stream->printf(";Optional arm solution specific settings:\nM665");
                    for(auto& i : options) {
                        gcode->stream->printf(" %c%1.4f", i.first, i.second);
                    }
                    gcode->stream->printf("\n");
                }
                gcode->mark_as_taken();
                break;
            }

            case 665: { // M665 set optional arm solution variables based on arm solution.
                gcode->mark_as_taken();
                // the parameter args could be any letter except S so ask solution what options it supports
                BaseSolution::arm_options_t options;
                if(arm_solution->get_optional(options)) {
                    for(auto& i : options) {
                        // foreach optional value
                        char c= i.first;
                        if(gcode->has_letter(c)) { // set new value
                            i.second= gcode->get_value(c);
                        }
                        // print all current values of supported options
                        gcode->stream->printf("%c: %8.4f ", i.first, i.second);
                        gcode->add_nl = true;
                    }
                    // set the new options
                    arm_solution->set_optional(options);
                }

                // set delta segments per second, not saved by M500
                if(gcode->has_letter('S')) {
                    this->delta_segments_per_second= gcode->get_value('S');
                }
                break;
            }
        }
    }

    if( this->motion_mode < 0)
        return;

   //Get parameters
    float target[3], offset[3];
    clear_vector(offset);

    memcpy(target, this->last_milestone, sizeof(target));    //default to last target

    for(char letter = 'I'; letter <= 'K'; letter++){
        if( gcode->has_letter(letter) ){
            offset[letter-'I'] = this->to_millimeters(gcode->get_value(letter));
        }
    }
    for(char letter = 'X'; letter <= 'Z'; letter++){
        if( gcode->has_letter(letter) ){
            target[letter-'X'] = this->to_millimeters(gcode->get_value(letter)) + ( this->absolute_mode ? 0 : target[letter-'X']);
        }
    }

    if( gcode->has_letter('F') )
    {
        if( this->motion_mode == MOTION_MODE_SEEK )
            this->seek_rate = this->to_millimeters( gcode->get_value('F') );
        else
            this->feed_rate = this->to_millimeters( gcode->get_value('F') );
    }

    //Perform any physical actions
    switch( next_action ){
        case NEXT_ACTION_DEFAULT:
            switch(this->motion_mode){
                case MOTION_MODE_CANCEL: break;
                case MOTION_MODE_SEEK  : this->append_line(gcode, target, this->seek_rate / seconds_per_minute ); break;
                case MOTION_MODE_LINEAR: this->append_line(gcode, target, this->feed_rate / seconds_per_minute ); break;
                case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC: this->compute_arc(gcode, offset, target ); break;
            }
            break;
    }

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    memcpy(this->last_milestone, target, sizeof(this->last_milestone)); // this->position[] = target[];

}

// We received a new gcode, and one of the functions
// determined the distance for that given gcode. So now we can attach this gcode to the right block
// and continue
void Robot::distance_in_gcode_is_known(Gcode* gcode){

    //If the queue is empty, execute immediatly, otherwise attach to the last added block
    THEKERNEL->conveyor->append_gcode(gcode);
}

// Reset the position for all axes ( used in homing and G92 stuff )
void Robot::reset_axis_position(float position, int axis) {
    this->last_milestone[axis] = position;

    float actuator_pos[3];
    arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);

    for (int i = 0; i < 3; i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);
}


// Convert target from millimeters to steps, and append this to the planner
void Robot::append_milestone( float target[], float rate_mm_s )
{
    float deltas[3];
    float unit_vec[3];
    float actuator_pos[3];
    float millimeters_of_travel;

    // find distance moved by each axis
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++)
        deltas[axis] = target[axis] - last_milestone[axis];

    // Compute how long this move moves, so we can attach it to the block for later use
    millimeters_of_travel = sqrtf( pow( deltas[X_AXIS], 2 ) +  pow( deltas[Y_AXIS], 2 ) +  pow( deltas[Z_AXIS], 2 ) );

    // find distance unit vector
    for (int i = 0; i < 3; i++)
        unit_vec[i] = deltas[i] / millimeters_of_travel;

    // Do not move faster than the configured cartesian limits
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++)
    {
        if ( max_speeds[axis] > 0 )
        {
            float axis_speed = fabs(unit_vec[axis] * rate_mm_s);

            if (axis_speed > max_speeds[axis])
                rate_mm_s *= ( max_speeds[axis] / axis_speed );
        }
    }

    // find actuator position given cartesian position
    arm_solution->cartesian_to_actuator( target, actuator_pos );

    // check per-actuator speed limits
    for (int actuator = 0; actuator <= 2; actuator++)
    {
        float actuator_rate  = fabs(actuator_pos[actuator] - actuators[actuator]->last_milestone_mm) * rate_mm_s / millimeters_of_travel;

        if (actuator_rate > actuators[actuator]->max_rate)
            rate_mm_s *= (actuators[actuator]->max_rate / actuator_rate);
    }

    // Append the block to the planner
    THEKERNEL->planner->append_block( actuator_pos, rate_mm_s, millimeters_of_travel, unit_vec );

    // Update the last_milestone to the current target for the next time we use last_milestone
    memcpy(this->last_milestone, target, sizeof(this->last_milestone)); // this->last_milestone[] = target[];

}

// Append a move to the queue ( cutting it into segments if needed )
void Robot::append_line(Gcode* gcode, float target[], float rate_mm_s ){

    // Find out the distance for this gcode
    gcode->millimeters_of_travel = pow( target[X_AXIS]-this->last_milestone[X_AXIS], 2 ) +  pow( target[Y_AXIS]-this->last_milestone[Y_AXIS], 2 ) +  pow( target[Z_AXIS]-this->last_milestone[Z_AXIS], 2 );

    // We ignore non-moves ( for example, extruder moves are not XYZ moves )
    if( gcode->millimeters_of_travel < 1e-8F ){
        return;
    }

    gcode->millimeters_of_travel = sqrtf(gcode->millimeters_of_travel);

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // We cut the line into smaller segments. This is not usefull in a cartesian robot, but necessary for robots with rotational axes.
    // In cartesian robot, a high "mm_per_line_segment" setting will prevent waste.
    // In delta robots either mm_per_line_segment can be used OR delta_segments_per_second The latter is more efficient and avoids splitting fast long lines into very small segments, like initial z move to 0, it is what Johanns Marlin delta port does
    uint16_t segments;

    if(this->delta_segments_per_second > 1.0F) {
        // enabled if set to something > 1, it is set to 0.0 by default
        // segment based on current speed and requested segments per second
        // the faster the travel speed the fewer segments needed
        // NOTE rate is mm/sec and we take into account any speed override
        float seconds = gcode->millimeters_of_travel / rate_mm_s;
        segments= max(1, ceil(this->delta_segments_per_second * seconds));
        // TODO if we are only moving in Z on a delta we don't really need to segment at all

    }else{
        if(this->mm_per_line_segment == 0.0F){
            segments= 1; // don't split it up
        }else{
            segments = ceil( gcode->millimeters_of_travel/ this->mm_per_line_segment);
        }
    }

    if (segments > 1)
    {
        // A vector to keep track of the endpoint of each segment
        float segment_delta[3];
        float segment_end[3];

        // How far do we move each segment?
        for (int i = X_AXIS; i <= Z_AXIS; i++)
            segment_delta[i] = (target[i] - last_milestone[i]) / segments;

        // segment 0 is already done - it's the end point of the previous move so we start at segment 1
        // We always add another point after this loop so we stop at segments-1, ie i < segments
        for (int i = 1; i < segments; i++)
        {
            for(int axis=X_AXIS; axis <= Z_AXIS; axis++ )
                segment_end[axis] = last_milestone[axis] + segment_delta[axis];

            // Append the end of this segment to the queue
            this->append_milestone(segment_end, rate_mm_s);
        }
    }

    // Append the end of this full move to the queue
    this->append_milestone(target, rate_mm_s);

    // if adding these blocks didn't start executing, do that now
    THEKERNEL->conveyor->ensure_running();
}


// Append an arc to the queue ( cutting it into segments as needed )
void Robot::append_arc(Gcode* gcode, float target[], float offset[], float radius, bool is_clockwise ){

    // Scary math
    float center_axis0 = this->last_milestone[this->plane_axis_0] + offset[this->plane_axis_0];
    float center_axis1 = this->last_milestone[this->plane_axis_1] + offset[this->plane_axis_1];
    float linear_travel = target[this->plane_axis_2] - this->last_milestone[this->plane_axis_2];
    float r_axis0 = -offset[this->plane_axis_0]; // Radius vector from center to current location
    float r_axis1 = -offset[this->plane_axis_1];
    float rt_axis0 = target[this->plane_axis_0] - center_axis0;
    float rt_axis1 = target[this->plane_axis_1] - center_axis1;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
    if (angular_travel < 0) { angular_travel += 2*M_PI; }
    if (is_clockwise) { angular_travel -= 2*M_PI; }

    // Find the distance for this gcode
    gcode->millimeters_of_travel = hypotf(angular_travel*radius, fabs(linear_travel));

    // We don't care about non-XYZ moves ( for example the extruder produces some of those )
    if( gcode->millimeters_of_travel < 0.0001F ){ return; }

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // Figure out how many segments for this gcode
    uint16_t segments = floor(gcode->millimeters_of_travel/this->mm_per_arc_segment);

    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = linear_travel/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi) cos(phi] * r ;
    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all float numbers are single precision on the Arduino. (True float precision will not have
    round off issues for CNC applications.) Single precision error can accumulate to be greater than
    tool precision in some cases. Therefore, arc path correction is implemented.

    Small angle approximation may be used to reduce computation overhead further. This approximation
    holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
    theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
    to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
    numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
    issue for CNC machines with the single precision Arduino calculations.
    This approximation also allows mc_arc to immediately insert a line segment into the planner
    without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
    a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
    This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1-0.5F*theta_per_segment*theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[3];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[this->plane_axis_2] = this->last_milestone[this->plane_axis_2];

    for (i = 1; i<segments; i++) { // Increment (segments-1)

        if (count < this->arc_correction ) {
          // Apply vector rotation matrix
          r_axisi = r_axis0*sin_T + r_axis1*cos_T;
          r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
          r_axis1 = r_axisi;
          count++;
        } else {
          // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
          // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
          cos_Ti = cosf(i*theta_per_segment);
          sin_Ti = sinf(i*theta_per_segment);
          r_axis0 = -offset[this->plane_axis_0]*cos_Ti + offset[this->plane_axis_1]*sin_Ti;
          r_axis1 = -offset[this->plane_axis_0]*sin_Ti - offset[this->plane_axis_1]*cos_Ti;
          count = 0;
        }

        // Update arc_target location
        arc_target[this->plane_axis_0] = center_axis0 + r_axis0;
        arc_target[this->plane_axis_1] = center_axis1 + r_axis1;
        arc_target[this->plane_axis_2] += linear_per_segment;

        // Append this segment to the queue
        this->append_milestone(arc_target, this->feed_rate / seconds_per_minute);

    }

    // Ensure last segment arrives at target location.
    this->append_milestone(target, this->feed_rate / seconds_per_minute);
}

// Do the math for an arc and add it to the queue
void Robot::compute_arc(Gcode* gcode, float offset[], float target[]){

    // Find the radius
    float radius = hypotf(offset[this->plane_axis_0], offset[this->plane_axis_1]);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( this->motion_mode == MOTION_MODE_CW_ARC ){ is_clockwise = true; }

    // Append arc
    this->append_arc(gcode, target, offset,  radius, is_clockwise );

}


float Robot::theta(float x, float y){
    float t = atanf(x/fabs(y));
    if (y>0) {return(t);} else {if (t>0){return(M_PI-t);} else {return(-M_PI-t);}}
}

void Robot::select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2){
    this->plane_axis_0 = axis_0;
    this->plane_axis_1 = axis_1;
    this->plane_axis_2 = axis_2;
}


