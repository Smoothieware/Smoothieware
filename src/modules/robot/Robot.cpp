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
#include "Player.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "../communication/utils/Gcode.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"

Robot::Robot(){
    this->inch_mode = false;
    this->absolute_mode = true;
    this->motion_mode =  MOTION_MODE_SEEK; 
    this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    clear_vector(this->current_position);
    clear_vector(this->last_milestone); 
}

//Called when the module has just been loaded
void Robot::on_module_loaded() {
    this->arm_solution = new CartesianSolution(this->kernel->config);
    this->register_for_event(ON_GCODE_RECEIVED);

    // Configuration
    this->on_config_reload(this);
}

void Robot::on_config_reload(void* argument){
    this->feed_rate =           this->kernel->config->value(default_feed_rate_checksum  )->by_default(100)->as_number()/60; 
    this->seek_rate =           this->kernel->config->value(default_seek_rate_checksum  )->by_default(100)->as_number()/60;
    this->mm_per_line_segment = this->kernel->config->value(mm_per_line_segment_checksum)->by_default(0.1)->as_number();
    this->mm_per_arc_segment  = this->kernel->config->value(mm_per_arc_segment_checksum )->by_default(10 )->as_number();
    this->arc_correction      = this->kernel->config->value(arc_correction_checksum     )->by_default(5  )->as_number();
    this->max_speeds[X_AXIS]  = this->kernel->config->value(x_axis_max_speed_checksum   )->by_default(0  )->as_number();
    this->max_speeds[Y_AXIS]  = this->kernel->config->value(y_axis_max_speed_checksum   )->by_default(0  )->as_number();
    this->max_speeds[Z_AXIS]  = this->kernel->config->value(z_axis_max_speed_checksum   )->by_default(0  )->as_number();
}

//A GCode has been received
void Robot::on_gcode_received(void * argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    gcode->call_on_gcode_execute_event_immediatly = false; 
    gcode->on_gcode_execute_event_called = false;
    //If the queue is empty, execute immediatly, otherwise attach to the last added block
    if( this->kernel->player->queue.size() == 0 ){
        gcode->call_on_gcode_execute_event_immediatly = true;
        this->execute_gcode(gcode);
        if( gcode->on_gcode_execute_event_called == false ){
            this->kernel->call_event(ON_GCODE_EXECUTE, gcode ); 
        }
    }else{
        Block* block = this->kernel->player->queue.get_ref( this->kernel->player->queue.size() - 1 );
        this->execute_gcode(gcode);
        block->append_gcode(gcode);
    }
    
}


//See if the current Gcode line has some orders for us
void Robot::execute_gcode(Gcode* gcode){
    
    //Temp variables, constant properties are stored in the object
    uint8_t next_action = NEXT_ACTION_DEFAULT;
    this->motion_mode = -1;

   //G-letter Gcodes are mostly what the Robot module is interrested in, other modules also catch the gcode event and do stuff accordingly
   if( gcode->has_letter('G')){
       switch( (int) gcode->get_value('G') ){
           case 0: this->motion_mode = MOTION_MODE_SEEK; break;
           case 1: this->motion_mode = MOTION_MODE_LINEAR; break;
           case 2: this->motion_mode = MOTION_MODE_CW_ARC; break;
           case 3: this->motion_mode = MOTION_MODE_CCW_ARC; break;
           case 17: this->select_plane(X_AXIS, Y_AXIS, Z_AXIS); break;
           case 18: this->select_plane(X_AXIS, Z_AXIS, Y_AXIS); break;
           case 19: this->select_plane(Y_AXIS, Z_AXIS, X_AXIS); break;
           case 20:this->inch_mode = true; break;
           case 21:this->inch_mode = false; break;
           case 90:this->absolute_mode = true; break;
           case 91:this->absolute_mode = false; break;
       } 
    }else{ return; }
    
   //Get parameters
    double target[3], offset[3];
    clear_vector(target); clear_vector(offset); 
    
    memcpy(target, this->current_position, sizeof(target));    //default to last target
    
    for(char letter = 'I'; letter <= 'K'; letter++){ if( gcode->has_letter(letter) ){ offset[letter-'I'] = this->to_millimeters(gcode->get_value(letter));                                                    } }     
    for(char letter = 'X'; letter <= 'Z'; letter++){ if( gcode->has_letter(letter) ){ target[letter-'X'] = this->to_millimeters(gcode->get_value(letter)) + ( this->absolute_mode ? 0 : target[letter-'X']);  } }     
    
    if( gcode->has_letter('F') ){ if( this->motion_mode == MOTION_MODE_SEEK ){ this->seek_rate = this->to_millimeters( gcode->get_value('F') ) / 60; }else{ this->feed_rate = this->to_millimeters( gcode->get_value('F') ) / 60; } }
   
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

// Convert target from millimeters to steps, and append this to the planner
void Robot::append_milestone( double target[], double rate ){
    int steps[3]; //Holds the result of the conversion
    
    this->arm_solution->millimeters_to_steps( target, steps );
    
    double deltas[3];
    for(int axis=X_AXIS;axis<=Z_AXIS;axis++){deltas[axis]=target[axis]-this->last_milestone[axis];}

    
    double millimeters_of_travel = sqrt( pow( deltas[X_AXIS], 2 ) +  pow( deltas[Y_AXIS], 2 ) +  pow( deltas[Z_AXIS], 2 ) );      
    
    double duration = 0;
    if( rate > 0 ){ duration = millimeters_of_travel / rate; }

    for(int axis=X_AXIS;axis<=Z_AXIS;axis++){
        if( this->max_speeds[axis] > 0 ){ 
            double axis_speed = ( fabs(deltas[axis]) / ( millimeters_of_travel / rate )) * 60; 
            if( axis_speed > this->max_speeds[axis] ){ 
                rate = rate * ( this->max_speeds[axis] / axis_speed ); 
            }
        }
    }

    this->kernel->planner->append_block( steps, rate*60, millimeters_of_travel, deltas ); 

    memcpy(this->last_milestone, target, sizeof(double)*3); // this->last_milestone[] = target[]; 

}

void Robot::append_line(Gcode* gcode, double target[], double rate ){


    // We cut the line into smaller segments. This is not usefull in a cartesian robot, but necessary for robots with rotational axes. 
    // In cartesian robot, a high "mm_per_line_segment" setting will prevent waste.
    gcode->millimeters_of_travel = sqrt( pow( target[X_AXIS]-this->current_position[X_AXIS], 2 ) +  pow( target[Y_AXIS]-this->current_position[Y_AXIS], 2 ) +  pow( target[Z_AXIS]-this->current_position[Z_AXIS], 2 ) ); 

    if( gcode->call_on_gcode_execute_event_immediatly == true ){
            this->kernel->call_event(ON_GCODE_EXECUTE, gcode ); 
            gcode->on_gcode_execute_event_called = true;
    }

    if (gcode->millimeters_of_travel == 0.0) { 
        this->append_milestone(this->current_position, 0.0);
        return; 
    }

    uint16_t segments = ceil( gcode->millimeters_of_travel/ this->mm_per_line_segment); 
    // A vector to keep track of the endpoint of each segment
    double temp_target[3];
    //Initialize axes
    memcpy( temp_target, this->current_position, sizeof(double)*3); // temp_target[] = this->current_position[];  

    //For each segment
    for( int i=0; i<segments-1; i++ ){
        for(int axis=X_AXIS; axis <= Z_AXIS; axis++ ){ temp_target[axis] += ( target[axis]-this->current_position[axis] )/segments; }        
        this->append_milestone(temp_target, rate); 
    }
    this->append_milestone(target, rate);
}


void Robot::append_arc(Gcode* gcode, double target[], double offset[], double radius, bool is_clockwise ){

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

    gcode->millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));

    if( gcode->call_on_gcode_execute_event_immediatly == true ){
            this->kernel->call_event(ON_GCODE_EXECUTE, gcode ); 
            gcode->on_gcode_execute_event_called = true;
    }

    if (gcode->millimeters_of_travel == 0.0) { 
        this->append_milestone(this->current_position, 0.0);
        return; 
    }
 
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
        this->append_milestone(arc_target, this->feed_rate);

    }
    // Ensure last segment arrives at target location.
    this->append_milestone(target, this->feed_rate);
}


void Robot::compute_arc(Gcode* gcode, double offset[], double target[]){

    // Find the radius
    double radius = hypot(offset[this->plane_axis_0], offset[this->plane_axis_1]);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( this->motion_mode == MOTION_MODE_CW_ARC ){ is_clockwise = true; } 

    // Append arc
    this->append_arc(gcode, target, offset,  radius, is_clockwise );

}


// Convert from inches to millimeters ( our internal storage unit ) if needed
inline double Robot::to_millimeters( double value ){
        return this->inch_mode ? value/25.4 : value; 
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


