/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <string>
using std::string;
#include <math.h>
#include "Planner.h"
#include "Conveyor.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "../communication/utils/Gcode.h"
#include "PublicDataRequest.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"
#include "arm_solutions/RotatableCartesianSolution.h"
#include "arm_solutions/RostockSolution.h"
#include "arm_solutions/JohannKosselSolution.h"
#include "arm_solutions/HBotSolution.h"

// The Robot converts GCodes into actual movements, and then adds them to the Planner, which passes them to the Conveyor so they can be added to the queue
// It takes care of cutting arcs into segments, same thing for line that are too long

Robot::Robot(){
    this->inch_mode = false;
    this->absolute_mode = true;
    this->motion_mode =  MOTION_MODE_SEEK;
    this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    clear_vector(this->current_position);
    clear_vector(this->last_milestone);
    this->arm_solution = NULL;
    seconds_per_minute = 60.0;
}

//Called when the module has just been loaded
void Robot::on_module_loaded() {
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Configuration
    this->on_config_reload(this);

    // Make our 3 StepperMotors
    this->alpha_stepper_motor  = this->kernel->step_ticker->add_stepper_motor( new StepperMotor(&alpha_step_pin,&alpha_dir_pin,&alpha_en_pin) );
    this->beta_stepper_motor   = this->kernel->step_ticker->add_stepper_motor( new StepperMotor(&beta_step_pin, &beta_dir_pin, &beta_en_pin ) );
    this->gamma_stepper_motor  = this->kernel->step_ticker->add_stepper_motor( new StepperMotor(&gamma_step_pin,&gamma_dir_pin,&gamma_en_pin) );

}

void Robot::on_config_reload(void* argument){

    // Arm solutions are used to convert positions in millimeters into position in steps for each stepper motor.
    // While for a cartesian arm solution, this is a simple multiplication, in other, less simple cases, there is some serious math to be done.
    // To make adding those solution easier, they have their own, separate object.
    // Here we read the config to find out which arm solution to use
    if (this->arm_solution) delete this->arm_solution;
    int solution_checksum = get_checksum(this->kernel->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());
    // Note checksums are not const expressions when in debug mode, so don't use switch
    if(solution_checksum == hbot_checksum || solution_checksum == corexy_checksum) {
        this->arm_solution = new HBotSolution(this->kernel->config);

    }else if(solution_checksum == rostock_checksum) {
        this->arm_solution = new RostockSolution(this->kernel->config);

    }else if(solution_checksum == kossel_checksum) {
        this->arm_solution = new JohannKosselSolution(this->kernel->config);

    }else if(solution_checksum ==  delta_checksum) {
        // place holder for now
        this->arm_solution = new RostockSolution(this->kernel->config);

    }else if(solution_checksum == rotatable_cartesian_checksum) {
        this->arm_solution = new RotatableCartesianSolution(this->kernel->config);

    }else if(solution_checksum == cartesian_checksum) {
        this->arm_solution = new CartesianSolution(this->kernel->config);

    }else{
        this->arm_solution = new CartesianSolution(this->kernel->config);
    }


    this->feed_rate           = this->kernel->config->value(default_feed_rate_checksum   )->by_default(100    )->as_number() / 60;
    this->seek_rate           = this->kernel->config->value(default_seek_rate_checksum   )->by_default(100    )->as_number() / 60;
    this->mm_per_line_segment = this->kernel->config->value(mm_per_line_segment_checksum )->by_default(0.0    )->as_number();
    this->delta_segments_per_second = this->kernel->config->value(delta_segments_per_second_checksum )->by_default(0.0    )->as_number();
    this->mm_per_arc_segment  = this->kernel->config->value(mm_per_arc_segment_checksum  )->by_default(0.5    )->as_number();
    this->arc_correction      = this->kernel->config->value(arc_correction_checksum      )->by_default(5      )->as_number();
    this->max_speeds[X_AXIS]  = this->kernel->config->value(x_axis_max_speed_checksum    )->by_default(60000  )->as_number();
    this->max_speeds[Y_AXIS]  = this->kernel->config->value(y_axis_max_speed_checksum    )->by_default(60000  )->as_number();
    this->max_speeds[Z_AXIS]  = this->kernel->config->value(z_axis_max_speed_checksum    )->by_default(300    )->as_number();
    this->alpha_step_pin.from_string( this->kernel->config->value(alpha_step_pin_checksum )->by_default("2.0"  )->as_string())->as_output();
    this->alpha_dir_pin.from_string(  this->kernel->config->value(alpha_dir_pin_checksum  )->by_default("0.5"  )->as_string())->as_output();
    this->alpha_en_pin.from_string(   this->kernel->config->value(alpha_en_pin_checksum   )->by_default("0.4"  )->as_string())->as_output();
    this->beta_step_pin.from_string(  this->kernel->config->value(beta_step_pin_checksum  )->by_default("2.1"  )->as_string())->as_output();
    this->gamma_step_pin.from_string( this->kernel->config->value(gamma_step_pin_checksum )->by_default("2.2"  )->as_string())->as_output();
    this->gamma_dir_pin.from_string(  this->kernel->config->value(gamma_dir_pin_checksum  )->by_default("0.20" )->as_string())->as_output();
    this->gamma_en_pin.from_string(   this->kernel->config->value(gamma_en_pin_checksum   )->by_default("0.19" )->as_string())->as_output();
    this->beta_dir_pin.from_string(   this->kernel->config->value(beta_dir_pin_checksum   )->by_default("0.11" )->as_string())->as_output();
    this->beta_en_pin.from_string(    this->kernel->config->value(beta_en_pin_checksum    )->by_default("0.10" )->as_string())->as_output();

}

void Robot::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        static double return_data;
        return_data= 100*this->seconds_per_minute/60;
        pdr->set_data_ptr(&return_data);
        pdr->set_taken();

    }else if(pdr->second_element_is(current_position_checksum)) {
        static double return_data[3];
        return_data[0]= from_millimeters(this->current_position[0]);
        return_data[1]= from_millimeters(this->current_position[1]);
        return_data[2]= from_millimeters(this->current_position[2]);

        pdr->set_data_ptr(&return_data);
        pdr->set_taken();
    }
}

void Robot::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        // NOTE do not use this while printing!
        double t= *static_cast<double*>(pdr->get_data_ptr());
        // enforce minimum 10% speed
        if (t < 10.0) t= 10.0;

        this->seconds_per_minute= t * 0.6;
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
                memcpy(this->current_position, this->last_milestone, sizeof(double)*3); // current_position[] = last_milestone[];
                this->arm_solution->millimeters_to_steps(this->current_position, this->kernel->planner->position);
                gcode->mark_as_taken();
                return; // TODO: Wait until queue empty
           }
       }
   }else if( gcode->has_m){
     switch( gcode->m ){
            case 92: // M92 - set steps per mm
                double steps[3];
                this->arm_solution->get_steps_per_millimeter(steps);
                if (gcode->has_letter('X'))
                    steps[0] = this->to_millimeters(gcode->get_value('X'));
                if (gcode->has_letter('Y'))
                    steps[1] = this->to_millimeters(gcode->get_value('Y'));
                if (gcode->has_letter('Z'))
                    steps[2] = this->to_millimeters(gcode->get_value('Z'));
                if (gcode->has_letter('F'))
                    seconds_per_minute = gcode->get_value('F');
                this->arm_solution->set_steps_per_millimeter(steps);
                // update current position in steps
                this->arm_solution->millimeters_to_steps(this->current_position, this->kernel->planner->position);
                gcode->stream->printf("X:%g Y:%g Z:%g F:%g ", steps[0], steps[1], steps[2], seconds_per_minute);
                gcode->add_nl = true;
                gcode->mark_as_taken();
                return;
            case 114: gcode->stream->printf("C: X:%1.3f Y:%1.3f Z:%1.3f ",
                                                 from_millimeters(this->current_position[0]),
                                                 from_millimeters(this->current_position[1]),
                                                 from_millimeters(this->current_position[2]));
                gcode->add_nl = true;
                gcode->mark_as_taken();
                return;
            // case 204: // M204 Snnn - set acceleration to nnn, NB only Snnn is currently supported
            //     gcode->mark_as_taken();
            //     if (gcode->has_letter('S'))
            //     {
            //         double acc= gcode->get_value('S') * 60 * 60; // mm/min^2
            //         // enforce minimum
            //         if (acc < 1.0)
            //             acc = 1.0;
            //         this->kernel->planner->acceleration= acc;
            //     }
            //     break;

            case 220: // M220 - speed override percentage
                gcode->mark_as_taken();
                if (gcode->has_letter('S'))
                {
                    double factor = gcode->get_value('S');
                    // enforce minimum 10% speed
                    if (factor < 10.0)
                        factor = 10.0;
                    seconds_per_minute = factor * 0.6;
                }
                break;
        }
   }
    if( this->motion_mode < 0)
        return;

   //Get parameters
    double target[3], offset[3];
    clear_vector(target); clear_vector(offset);

    memcpy(target, this->current_position, sizeof(target));    //default to last target

    for(char letter = 'I'; letter <= 'K'; letter++){ if( gcode->has_letter(letter) ){ offset[letter-'I'] = this->to_millimeters(gcode->get_value(letter));                                                    } }
    for(char letter = 'X'; letter <= 'Z'; letter++){ if( gcode->has_letter(letter) ){ target[letter-'X'] = this->to_millimeters(gcode->get_value(letter)) + ( this->absolute_mode ? 0 : target[letter-'X']);  } }

    if( gcode->has_letter('F') )
    {
        if( this->motion_mode == MOTION_MODE_SEEK )
            this->seek_rate = this->to_millimeters( gcode->get_value('F') ) / 60.0;
        else
            this->feed_rate = this->to_millimeters( gcode->get_value('F') ) / 60.0;
    }

    //Perform any physical actions
    switch( next_action ){
        case NEXT_ACTION_DEFAULT:
            switch(this->motion_mode){
                case MOTION_MODE_CANCEL: break;
                case MOTION_MODE_SEEK  : this->append_line(gcode, target, this->seek_rate ); break;
                case MOTION_MODE_LINEAR: this->append_line(gcode, target, this->feed_rate ); break;
                case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC: this->compute_arc(gcode, offset, target ); break;
            }
            break;
    }

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    memcpy(this->current_position, target, sizeof(double)*3); // this->position[] = target[];

}

// We received a new gcode, and one of the functions
// determined the distance for that given gcode. So now we can attach this gcode to the right block
// and continue
void Robot::distance_in_gcode_is_known(Gcode* gcode){

    //If the queue is empty, execute immediatly, otherwise attach to the last added block
    if( this->kernel->conveyor->queue.size() == 0 ){
        this->kernel->call_event(ON_GCODE_EXECUTE, gcode );
    }else{
        Block* block = this->kernel->conveyor->queue.get_ref( this->kernel->conveyor->queue.size() - 1 );
        block->append_gcode(gcode);
    }

}

// Reset the position for all axes ( used in homing and G92 stuff )
void Robot::reset_axis_position(double position, int axis) {
    this->last_milestone[axis] = this->current_position[axis] = position;
    this->arm_solution->millimeters_to_steps(this->current_position, this->kernel->planner->position);
}


// Convert target from millimeters to steps, and append this to the planner
void Robot::append_milestone( double target[], double rate ){
    int steps[3]; //Holds the result of the conversion

    // We use an arm solution object so exotic arm solutions can be used and neatly abstracted
    this->arm_solution->millimeters_to_steps( target, steps );

    double deltas[3];
    for(int axis=X_AXIS;axis<=Z_AXIS;axis++){deltas[axis]=target[axis]-this->last_milestone[axis];}

    // Compute how long this move moves, so we can attach it to the block for later use
    double millimeters_of_travel = sqrt( pow( deltas[X_AXIS], 2 ) +  pow( deltas[Y_AXIS], 2 ) +  pow( deltas[Z_AXIS], 2 ) );

    // Do not move faster than the configured limits
    for(int axis=X_AXIS;axis<=Z_AXIS;axis++){
        if( this->max_speeds[axis] > 0 ){
            double axis_speed = ( fabs(deltas[axis]) / ( millimeters_of_travel / rate )) * seconds_per_minute;
            if( axis_speed > this->max_speeds[axis] ){
                rate = rate * ( this->max_speeds[axis] / axis_speed );
            }
        }
    }

    // Append the block to the planner
    this->kernel->planner->append_block( steps, rate * seconds_per_minute, millimeters_of_travel, deltas );

    // Update the last_milestone to the current target for the next time we use last_milestone
    memcpy(this->last_milestone, target, sizeof(double)*3); // this->last_milestone[] = target[];

}

// Append a move to the queue ( cutting it into segments if needed )
void Robot::append_line(Gcode* gcode, double target[], double rate ){

    // Find out the distance for this gcode
    gcode->millimeters_of_travel = sqrt( pow( target[X_AXIS]-this->current_position[X_AXIS], 2 ) +  pow( target[Y_AXIS]-this->current_position[Y_AXIS], 2 ) +  pow( target[Z_AXIS]-this->current_position[Z_AXIS], 2 ) );

    // We ignore non-moves ( for example, extruder moves are not XYZ moves )
    if( gcode->millimeters_of_travel < 0.0001 ){ return; }

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // We cut the line into smaller segments. This is not usefull in a cartesian robot, but necessary for robots with rotational axes.
    // In cartesian robot, a high "mm_per_line_segment" setting will prevent waste.
    // In delta robots either mm_per_line_segment can be used OR delta_segments_per_second The latter is more efficient and avoids splitting fast long lines into very small segments, like initial z move to 0, it is what Johanns Marlin delta port does
    uint16_t segments;

    if(this->delta_segments_per_second > 1.0) {
        // enabled if set to something > 1, it is set to 0.0 by default
        // segment based on current speed and requested segments per second
        // the faster the travel speed the fewer segments needed
        // NOTE rate is mm/sec and we take into account any speed override
        float seconds = 60.0/seconds_per_minute * gcode->millimeters_of_travel / rate;
        segments= max(1, ceil(this->delta_segments_per_second * seconds));
        // TODO if we are only moving in Z on a delta we don't really need to segment at all

    }else{
        if(this->mm_per_line_segment == 0.0){
            segments= 1; // don't split it up
        }else{
            segments = ceil( gcode->millimeters_of_travel/ this->mm_per_line_segment);
        }
    }

    // A vector to keep track of the endpoint of each segment
    double temp_target[3];
    //Initialize axes
    memcpy( temp_target, this->current_position, sizeof(double)*3); // temp_target[] = this->current_position[];

    //For each segment
    for( int i=0; i<segments-1; i++ ){
        for(int axis=X_AXIS; axis <= Z_AXIS; axis++ ){ temp_target[axis] += ( target[axis]-this->current_position[axis] )/segments; }
        // Append the end of this segment to the queue
        this->append_milestone(temp_target, rate);
    }

    // Append the end of this full move to the queue
    this->append_milestone(target, rate);
}


// Append an arc to the queue ( cutting it into segments as needed )
void Robot::append_arc(Gcode* gcode, double target[], double offset[], double radius, bool is_clockwise ){

    // Scary math
    double center_axis0 = this->current_position[this->plane_axis_0] + offset[this->plane_axis_0];
    double center_axis1 = this->current_position[this->plane_axis_1] + offset[this->plane_axis_1];
    double linear_travel = target[this->plane_axis_2] - this->current_position[this->plane_axis_2];
    double r_axis0 = -offset[this->plane_axis_0]; // Radius vector from center to current location
    double r_axis1 = -offset[this->plane_axis_1];
    double rt_axis0 = target[this->plane_axis_0] - center_axis0;
    double rt_axis1 = target[this->plane_axis_1] - center_axis1;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    double angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
    if (angular_travel < 0) { angular_travel += 2*M_PI; }
    if (is_clockwise) { angular_travel -= 2*M_PI; }

    // Find the distance for this gcode
    gcode->millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));

    // We don't care about non-XYZ moves ( for example the extruder produces some of those )
    if( gcode->millimeters_of_travel < 0.0001 ){ return; }

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // Figure out how many segments for this gcode
    uint16_t segments = floor(gcode->millimeters_of_travel/this->mm_per_arc_segment);

    double theta_per_segment = angular_travel/segments;
    double linear_per_segment = linear_travel/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi) cos(phi] * r ;
    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all double numbers are single precision on the Arduino. (True double precision will not have
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
    double cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
    double sin_T = theta_per_segment;

    double arc_target[3];
    double sin_Ti;
    double cos_Ti;
    double r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[this->plane_axis_2] = this->current_position[this->plane_axis_2];

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
          cos_Ti = cos(i*theta_per_segment);
          sin_Ti = sin(i*theta_per_segment);
          r_axis0 = -offset[this->plane_axis_0]*cos_Ti + offset[this->plane_axis_1]*sin_Ti;
          r_axis1 = -offset[this->plane_axis_0]*sin_Ti - offset[this->plane_axis_1]*cos_Ti;
          count = 0;
        }

        // Update arc_target location
        arc_target[this->plane_axis_0] = center_axis0 + r_axis0;
        arc_target[this->plane_axis_1] = center_axis1 + r_axis1;
        arc_target[this->plane_axis_2] += linear_per_segment;

        // Append this segment to the queue
        this->append_milestone(arc_target, this->feed_rate);

    }

    // Ensure last segment arrives at target location.
    this->append_milestone(target, this->feed_rate);
}

// Do the math for an arc and add it to the queue
void Robot::compute_arc(Gcode* gcode, double offset[], double target[]){

    // Find the radius
    double radius = hypot(offset[this->plane_axis_0], offset[this->plane_axis_1]);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( this->motion_mode == MOTION_MODE_CW_ARC ){ is_clockwise = true; }

    // Append arc
    this->append_arc(gcode, target, offset,  radius, is_clockwise );

}


double Robot::theta(double x, double y){
    double t = atan(x/fabs(y));
    if (y>0) {return(t);} else {if (t>0){return(M_PI-t);} else {return(-M_PI-t);}}
}

void Robot::select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2){
    this->plane_axis_0 = axis_0;
    this->plane_axis_1 = axis_1;
    this->plane_axis_2 = axis_2;
}


