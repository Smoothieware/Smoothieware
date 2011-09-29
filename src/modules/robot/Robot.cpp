/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <string>
using std::string;
#include "mbed.h"
#include <math.h>
#include "Planner.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "../communication/utils/Gcode.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"

Robot::Robot(){
    this->inch_mode = true;
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
    this->feed_rate =           this->kernel->config->get(default_feed_rate_checksum)/60; 
    this->seek_rate =           this->kernel->config->get(default_seek_rate_checksum)/60;
    this->mm_per_line_segment = this->kernel->config->get(mm_per_line_segment_checksum);
    this->mm_per_arc_segment  = this->kernel->config->get(mm_per_arc_segment_checksum); 
}

//A GCode has been received
void Robot::on_gcode_received(void * argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    this->execute_gcode(gcode);
    this->kernel->planner->attach_gcode_to_queue(gcode);
}

//See if the current Gcode line has some orders for us
void Robot::execute_gcode(Gcode* gcode){
    
    //Temp variables, constant properties are stored in the object
    uint8_t next_action = NEXT_ACTION_DEFAULT;

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
    }
    
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
                case MOTION_MODE_SEEK  : this->append_line(target, this->seek_rate ); break;
                case MOTION_MODE_LINEAR: this->append_line(target, this->feed_rate ); break;
                case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC: this->compute_arc(offset, target ); break; 
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

    double millimeters_of_travel = sqrt( pow( target[X_AXIS]-this->last_milestone[X_AXIS], 2 ) +  pow( target[Y_AXIS]-this->last_milestone[Y_AXIS], 2 ) +  pow( target[Z_AXIS]-this->last_milestone[Z_AXIS], 2 ) );      
    if( millimeters_of_travel < 0.001 ){ return; } 
    double duration = millimeters_of_travel / rate;

    double speeds[3];
    for(int axis=X_AXIS;axis<=Z_AXIS;axis++){speeds[axis]=(target[axis]-this->last_milestone[axis])/duration;}

    this->kernel->planner->append_block( steps, rate, millimeters_of_travel, speeds ); 

    memcpy(this->last_milestone, target, sizeof(double)*3); // this->position[] = target[]; 

}

void Robot::append_line(double target[], double rate ){

    // We cut the line into smaller segments. This is not usefull in a cartesian robot, but necessary for robots with rotational axes. 
    // In cartesian robot, a high "mm_per_line_segment" setting will prevent waste.
    double millimeters_of_travel = sqrt( pow( target[X_AXIS]-this->current_position[X_AXIS], 2 ) +  pow( target[Y_AXIS]-this->current_position[Y_AXIS], 2 ) +  pow( target[Z_AXIS]-this->current_position[Z_AXIS], 2 ) ); 
    if( millimeters_of_travel == 0.0 ){ return; }

    uint16_t segments = ceil( millimeters_of_travel/ this->mm_per_line_segment); 
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

void Robot::append_arc(double theta_start, double angular_travel, double radius, double depth, double rate){

    // The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
    // segment is configured in settings.mm_per_arc_segment.  
    double millimeters_of_travel = hypot(angular_travel*radius, labs(depth));
    if (millimeters_of_travel == 0.0) { return; }
    uint16_t segments = ceil(millimeters_of_travel/ this->mm_per_arc_segment );
    // The angular motion for each segment
    double theta_per_segment = angular_travel/segments;
    // The linear motion for each segment
    double linear_per_segment = depth/segments;
    // Compute the center of this circle
    double center_x = this->current_position[this->plane_axis_0]-sin(theta_start)*radius;
    double center_y = this->current_position[this->plane_axis_1]-cos(theta_start)*radius;
    // a vector to track the end point of each segment
    double target[3];

    // Initialize the linear axis
    target[this->plane_axis_2] = this->current_position[this->plane_axis_2];
    for (int i=0; i<segments; i++) { //NOTE: Major grbl diff
        target[this->plane_axis_2] += linear_per_segment;
        theta_start += theta_per_segment;
        target[this->plane_axis_0] = center_x+sin(theta_start)*radius;
        target[this->plane_axis_1] = center_y+cos(theta_start)*radius;
        this->append_milestone(target, rate);
    }

}


void Robot::compute_arc(double offset[], double target[]){
    /*
     This segment sets up an clockwise or counterclockwise arc from the current position to the target position around 
     the center designated by the offset vector. All theta-values measured in radians of deviance from the positive 
     y-axis. 

                        | <- theta == 0
                      * * *                
                    *       *                                               
                  *           *                                             
                  *     O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)                                          
                  *   /                                                     
                    C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))

    */
        
    // calculate the theta (angle) of the current point
    double theta_start = theta(-offset[this->plane_axis_0], -offset[this->plane_axis_1]);
    // calculate the theta (angle) of the target point
    double theta_end = theta(target[this->plane_axis_0] - offset[this->plane_axis_0] - this->current_position[this->plane_axis_0], target[this->plane_axis_1] - offset[this->plane_axis_1] - this->current_position[this->plane_axis_1]);
    //printf(" theta_start: %3.3f, theta_end: %3.3f \r\n", theta_start, theta_end);

    // ensure that the difference is positive so that we have clockwise travel
    if (theta_end <= theta_start) { theta_end += 2*M_PI; }
    double angular_travel = theta_end-theta_start;
    // Invert angular motion if the g-code wanted a counterclockwise arc
    if (this->motion_mode == MOTION_MODE_CCW_ARC) { angular_travel = angular_travel-2*M_PI; }
    // Find the radius
    double radius = hypot(offset[this->plane_axis_0], offset[this->plane_axis_1]);
    // Calculate the motion along the depth axis of the helix
    double depth = target[this->plane_axis_2]-this->current_position[this->plane_axis_2];
    // Trace the arc
    this->append_arc(theta_start, angular_travel, radius, depth, this->feed_rate);
    // Finish off with a line to make sure we arrive exactly where we think we are
    //this->append_line(target, this->feed_rate);
    this->append_milestone( target, this->feed_rate );
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


