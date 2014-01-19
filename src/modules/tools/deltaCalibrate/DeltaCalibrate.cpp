/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "DeltaCalibrate.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"


#define UP false
#define DOWN true
#define FAST true
#define SLOW false
#define DEG_TO_RAD (3.141592653589793/180)

#define DO_CALIBRATE_DELTA 1
#define CALIBRATE_AUTOSET 2
#define CALIBRATE_SILENT 4
#define CALIBRATE_QUIET 8
#define DO_CALIBRATE_PROBE 16

#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")
#define delta_homing_checksum                   CHECKSUM("delta_homing")

#define gamma_min_endstop_checksum       CHECKSUM("gamma_min_endstop")

#define alpha_trim_checksum              CHECKSUM("alpha_trim")
#define beta_trim_checksum               CHECKSUM("beta_trim")
#define gamma_trim_checksum              CHECKSUM("gamma_trim")

#define calibrate_lift_checksum          CHECKSUM("calibrate_lift")
#define calibrate_radius_checksum        CHECKSUM("calibrate_radius")
#define calibrate_probe_offset_checksum  CHECKSUM("calibrate_probe_offset")

// these values are in steps and should be deprecated
#define alpha_fast_homing_rate_checksum  CHECKSUM("alpha_fast_homing_rate")
#define alpha_slow_homing_rate_checksum  CHECKSUM("alpha_slow_homing_rate")
#define alpha_homing_retract_checksum    CHECKSUM("alpha_homing_retract")
#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")

// same as above but in user friendly mm/s and mm
#define alpha_fast_homing_rate_mm_checksum  CHECKSUM("alpha_fast_homing_rate_mm_s")
#define alpha_slow_homing_rate_mm_checksum  CHECKSUM("alpha_slow_homing_rate_mm_s")
#define alpha_homing_retract_mm_checksum    CHECKSUM("alpha_homing_retract_mm")
#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")

#define alpha_homing_direction_checksum  CHECKSUM("alpha_homing_direction")
#define beta_homing_direction_checksum   CHECKSUM("beta_homing_direction")
#define gamma_homing_direction_checksum  CHECKSUM("gamma_homing_direction")
#define home_to_max_checksum             CHECKSUM("home_to_max")
#define home_to_min_checksum             CHECKSUM("home_to_min")
#define gamma_min_checksum               CHECKSUM("gamma_min")
#define gamma_max_checksum               CHECKSUM("gamma_max")

#define alpha_steps_per_mm_checksum      CHECKSUM("alpha_steps_per_mm")

#define arm_solution_checksum                  CHECKSUM("arm_solution")
#define kossel_checksum                        CHECKSUM("kossel")


DeltaCalibrate::DeltaCalibrate()
{
    this->delta_calibrate_flags = 0;
}

void DeltaCalibrate::on_module_loaded()
{
    // Do not do anything if not enabled
    if ( THEKERNEL->config->value( endstops_module_enable_checksum )->by_default(true)->as_bool() == false ) {
        return;
    }

    // we directly access arm solution data to calculate calibration, currently only Kossel solution has the required methods
    if ( get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string()) != kossel_checksum ) {
        return;
    }

    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);

    // Take StepperMotor objects from Robot and keep them here
    this->steppers[0] = THEKERNEL->robot->alpha_stepper_motor;
    this->steppers[1] = THEKERNEL->robot->beta_stepper_motor;
    this->steppers[2] = THEKERNEL->robot->gamma_stepper_motor;

    // Settings
    this->on_config_reload(this);

}

// Get config
void DeltaCalibrate::on_config_reload(void *argument)
{
    this->probe_pin.from_string(         THEKERNEL->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();

    // we need to know steps per mm for M206, also use them for all settings
    this->steps_per_mm           =  THEKERNEL->config->value(alpha_steps_per_mm_checksum         )->as_number();

    this->fast_rates             =  THEKERNEL->config->value(alpha_fast_homing_rate_checksum     )->by_default(4000 )->as_number();
    this->slow_rates             =  THEKERNEL->config->value(alpha_slow_homing_rate_checksum     )->by_default(2000 )->as_number();
    this->retract_steps          =  THEKERNEL->config->value(alpha_homing_retract_checksum       )->by_default(400  )->as_number();

    // newer mm based config values override the old ones, convert to steps/mm and steps, defaults to what was set in the older config settings above
    this->fast_rates =    THEKERNEL->config->value(alpha_fast_homing_rate_mm_checksum )->by_default(this->fast_rates  / steps_per_mm)->as_number() * steps_per_mm;
    this->slow_rates =    THEKERNEL->config->value(alpha_slow_homing_rate_mm_checksum )->by_default(this->slow_rates  / steps_per_mm)->as_number() * steps_per_mm;
    this->retract_steps = THEKERNEL->config->value(alpha_homing_retract_mm_checksum   )->by_default(this->retract_steps / steps_per_mm)->as_number() * steps_per_mm;

    this->debounce_count  = THEKERNEL->config->value(endstop_debounce_count_checksum    )->by_default(0)->as_number();
    this->lift_steps = THEKERNEL->config->value(calibrate_lift_checksum )->by_default(10)->as_number() * steps_per_mm;
    this->calibrate_radius = THEKERNEL->config->value(calibrate_radius_checksum )->by_default(50)->as_number();
    this->calibrate_probe_offset = THEKERNEL->config->value(calibrate_probe_offset_checksum )->by_default(0)->as_number();

    // get homing direction and convert to boolean where true is home to min, and false is home to max
    int home_dir                    = get_checksum(THEKERNEL->config->value(alpha_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[0]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(beta_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[1]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(gamma_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[2]         = home_dir != home_to_max_checksum;

    this->homing_position_z        =  this->home_direction[2] ? THEKERNEL->config->value(gamma_min_checksum)->by_default(0)->as_number() : THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number();;

    this->is_delta                  =  THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();

    // endstop trim used by deltas to do soft adjusting, in mm, convert to steps, and negate depending on homing direction
    // eg on a delta homing to max, a negative trim value will move the carriage down, and a positive will move it up
    int dirx = (this->home_direction[0] ? 1 : -1);
    int diry = (this->home_direction[1] ? 1 : -1);
    int dirz = (this->home_direction[2] ? 1 : -1);
    this->trim[0] = THEKERNEL->config->value(alpha_trim_checksum )->by_default(0  )->as_number() * steps_per_mm * dirx;
    this->trim[1] = THEKERNEL->config->value(beta_trim_checksum  )->by_default(0  )->as_number() * steps_per_mm * diry;
    this->trim[2] = THEKERNEL->config->value(gamma_trim_checksum )->by_default(0  )->as_number() * steps_per_mm * dirz;
}

void DeltaCalibrate::wait_for_moves(){
    // Wait for moves to be done
    for( int i = 0; i <= 2; i++ ){
        while( this->steppers[i]->moving ){
            THEKERNEL->call_event(ON_IDLE);
        }
    }
}

uint32_t DeltaCalibrate::wait_for_ztouch(){
    bool running = true;
    unsigned int debounce = 0;
    uint32_t saved_steps = 0;
    while(running){
        running = false;
        THEKERNEL->call_event(ON_IDLE);
        if( this->probe_pin.get() ){  //probe contact 
            if( debounce < debounce_count ) { //debounce and loop
                debounce ++;
                running = true;
            } else { //fully debounced, stop all motors
                saved_steps = this->steppers[0]->stepped; //save the number of actually moved steps from alpha tower before tool counters get reset
                for( int i = 0; i <= 2; i++ ){ //TODO support more than 3 axis
                    this->steppers[i]->move(0,0);
                }
                return (saved_steps);
            }
        }else{
            // The endstop was not hit yet
            running = true;
            debounce = 0;
        }
    }

    return(0);
}

uint32_t DeltaCalibrate::do_probe(bool ret){
    uint32_t current_z_steps = 0;

    //probe current location
    move_all(DOWN,FAST,10000000);
    current_z_steps += wait_for_ztouch();

    move_all(UP,FAST,retract_steps); 
    wait_for_moves();
    current_z_steps -= retract_steps;

    move_all(DOWN,SLOW,10000000);
    current_z_steps += wait_for_ztouch();

    if (ret){
        //return to original position
        move_all(UP,FAST, current_z_steps); //alpha tower retract distance for all
        wait_for_moves();
    }

    return(current_z_steps);
}

void DeltaCalibrate::move_all(bool direction, bool speed, unsigned int steps){
    bool move_dir;
    for( int i = 0; i <= 2; i++ ){
        move_dir = (this->home_direction[i]^direction) != 0;
        if(speed){
            this->steppers[i]->set_speed(this->fast_rates);
        } else {
            this->steppers[i]->set_speed(this->slow_rates);
        }
        this->steppers[i]->move(move_dir,steps);
    }
}

// auto calibration routine for delta bots
void DeltaCalibrate::calibrate_delta(StreamOutput *stream){

    uint32_t current_z_steps = 0;
    uint32_t probe_steps = 0;

    uint32_t originHeight = 0;
    long tower1Height = 0;
    long tower2Height = 0;
    long tower3Height = 0;
    float targetX = 0;
    float targetY = 0;

    char buf[32];
    int buffered_length = 0;
    string g;

    float arm_radius = 0;

    // update machine geometry incase it has been modified since startup
    THEKERNEL->robot->arm_solution->get_optional('R', &arm_radius);

    // First wait for the queue to be empty
    THEKERNEL->conveyor->wait_for_empty_queue();
    // Enable the motors
    THEKERNEL->stepper->turn_enable_pins_on();
    
    //automatically home all home axis
    buffered_length = snprintf(buf, sizeof(buf), "G28");
    g.assign(buf, buffered_length);
    send_gcode(g);

    buffered_length = snprintf(buf, sizeof(buf), "G21");
    g.assign(buf, buffered_length);
    send_gcode(g);

    buffered_length = snprintf(buf, sizeof(buf), "G90");
    g.assign(buf, buffered_length);
    send_gcode(g);

    //TODO auto-deploy probe

    // deploy probe, check or abort
    if( probe_pin.get() ){
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }

    //probe min z height at 0,0
    current_z_steps = do_probe(false);
    originHeight = current_z_steps;

    //lift and reposition
    move_all(UP,FAST,lift_steps); //alpha tower retract distance for all
    wait_for_moves();
    current_z_steps -= lift_steps;

    // update planner position so we can make coordinated moves
    THEKERNEL->robot->reset_axis_position( homing_position_z - (current_z_steps / steps_per_mm) , 2);

    targetY = calibrate_radius * cos(240 * DEG_TO_RAD);
    targetX = calibrate_radius * sin(240 * DEG_TO_RAD);
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX,targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    THEKERNEL->conveyor->wait_for_empty_queue();


    // check probe retraction
    if( probe_pin.get() ){
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }

    //probe tower 1 
    probe_steps = do_probe(true);
    tower1Height = current_z_steps + probe_steps;

    targetY = calibrate_radius * cos(120 * DEG_TO_RAD);
    targetX = calibrate_radius * sin(120 * DEG_TO_RAD);
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX,targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    THEKERNEL->conveyor->wait_for_empty_queue();

    // check probe retraction
    if( probe_pin.get() ){
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }    

    //probe tower 2
    probe_steps = do_probe(true);
    tower2Height = current_z_steps + probe_steps; 

    targetY = calibrate_radius * cos(0 * DEG_TO_RAD);
    targetX = calibrate_radius * sin(0 * DEG_TO_RAD);
    
    buffered_length = snprintf(buf, sizeof(buf), "G0 X%f Y%f", targetX, targetY);
    g.assign(buf, buffered_length);
    send_gcode(g);
    THEKERNEL->conveyor->wait_for_empty_queue();    

    // check probe retraction
    if( this->probe_pin.get() ){
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }  

    //probe tower 3 
    probe_steps = do_probe(true);
    tower3Height = current_z_steps + probe_steps; 

    //TODO auto-retract probe

    //calculate 3 tower trim levels
    float centerAverage = (tower1Height + tower2Height + tower3Height)/3;
    long lowestTower = min(tower1Height,min(tower2Height,tower3Height));

    float multiplier =  (1.83606 *log(arm_radius*2)-9.76413)/(1.83606 *log(arm_radius+calibrate_radius)-9.76413);
    float t1Trim = ((tower1Height-lowestTower) / steps_per_mm) * multiplier;
    float t2Trim = ((tower2Height-lowestTower) / steps_per_mm) * multiplier;
    float t3Trim = ((tower3Height-lowestTower) / steps_per_mm) * multiplier;

    if ( (delta_calibrate_flags & (CALIBRATE_SILENT|CALIBRATE_QUIET) ) == 0 ){
        stream->printf("Probed Points:\r\n");
        stream->printf("X:%5.3f Y:%5.3f Z:%5.3f \r\n", -tower1Height/steps_per_mm, -tower2Height/steps_per_mm, -tower3Height/steps_per_mm); 
        stream->printf("Detected offsets:\r\n");
        stream->printf("Origin Offset: %5.3f\r\n", (centerAverage - originHeight)/steps_per_mm); 
        stream->printf("X:%5.3f Y:%5.3f Z:%5.3f \r\n", -t1Trim, -t2Trim, -t3Trim); 
    }

    t1Trim += trim[0]/steps_per_mm;
    t2Trim += trim[1]/steps_per_mm;
    t3Trim += trim[2]/steps_per_mm;
    float minTrim = min(t1Trim,min(t2Trim,t3Trim));
    t1Trim -= minTrim;
    t2Trim -= minTrim;
    t3Trim -= minTrim;
    float newDeltaRadius = arm_radius - ((centerAverage - originHeight)/steps_per_mm)/0.15;

    if ( (delta_calibrate_flags & CALIBRATE_SILENT) == 0 ) {
        stream->printf("Calibrated Values:\r\n");
        stream->printf("Origin Height:%5.3f\r\n",  (originHeight/steps_per_mm) + calibrate_probe_offset); 
        stream->printf("Delta Radius:%5.3f\r\n",newDeltaRadius); 
        stream->printf("X:%5.3f Y:%5.3f Z:%5.3f \r\n", -t1Trim, -t2Trim, -t3Trim); 
    }

    // apply values
    if ( (delta_calibrate_flags & CALIBRATE_AUTOSET) > 0 ){

        buffered_length = snprintf(buf, sizeof(buf), "M666 X%5.3f Y%5.3f Z%5.3f", -t1Trim, -t2Trim, -t3Trim );
        g.assign(buf, buffered_length);
        send_gcode(g);

        buffered_length = snprintf(buf, sizeof(buf), "M665 R%5.3f Z%5.3f", newDeltaRadius, (( originHeight/steps_per_mm) + calibrate_probe_offset) );
        g.assign(buf, buffered_length);
        send_gcode(g);

        stream->printf("Calibration values have been changed in memory, but your config file has not been modified.\r\n");
    }
}

void DeltaCalibrate::calibrate_zprobe_offset(StreamOutput *stream){

    uint32_t current_z_steps = 0;
    long originHeight = 0;

    // First wait for the queue to be empty
    THEKERNEL->conveyor->wait_for_empty_queue();
    // Enable the motors
    THEKERNEL->stepper->turn_enable_pins_on();

    if( !probe_pin.get() ){ 
        //probe not deployed - abort
        stream->printf("Z-Probe not activated, Aborting!\r\n");
        return;
    }

    move_all(UP,FAST,lift_steps); //alpha tower retract distance for all
    wait_for_moves();
    current_z_steps = 0;

    if( probe_pin.get() ){ 
        //probe not deployed - abort
        stream->printf("Z-Probe not deployed, aborting!\r\n");
        return;
    }

    move_all(DOWN,SLOW,10000000);
    current_z_steps += wait_for_ztouch();
    originHeight = lift_steps - current_z_steps;

    //lift and reposition
    move_all(UP,FAST,lift_steps); //alpha tower retract distance for all
    wait_for_moves();

    //calculate
    if ( (delta_calibrate_flags & CALIBRATE_SILENT) == 0 ) {
        stream->printf("Calibrated Values:\r\n");
        stream->printf("Probe Offset:%5.3f\r\n",  (originHeight/steps_per_mm)); 
    }
    // apply values
    if ( (delta_calibrate_flags & CALIBRATE_AUTOSET) > 0 ){
        calibrate_probe_offset = originHeight/steps_per_mm; 
        stream->printf("Probe offset has been changed in memory, but your config file has not been modified.\r\n");
    }
}

// Start homing sequences by response to GCode commands
void DeltaCalibrate::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if ( gcode->has_g) {
        if (gcode->g == 32 )
        {
            if (is_delta) {
                gcode->mark_as_taken();
                delta_calibrate_flags = 0;
                delta_calibrate_flags |= DO_CALIBRATE_DELTA;

                if (gcode->has_letter('Q')) {
                    delta_calibrate_flags |= CALIBRATE_QUIET;
                }
                if (gcode->has_letter('S')) {
                    delta_calibrate_flags |= CALIBRATE_SILENT;
                }
                if (gcode->has_letter('D')) {
                    //disable auto-apply values
                } else {
                    delta_calibrate_flags |= CALIBRATE_AUTOSET;
                }
                if (gcode->has_letter('R')) {
                    calibrate_radius = gcode->get_value('R');
                }
                if (gcode->has_letter('L')) {
                    float l = gcode->get_value('L');
                    lift_steps = l * steps_per_mm;
                }
                calibrate_delta(gcode->stream);
            }  

        } else if (gcode->g == 31 )
        {
            gcode->mark_as_taken();
            delta_calibrate_flags = 0;
            delta_calibrate_flags |= DO_CALIBRATE_PROBE;

            if (gcode->has_letter('S')) {
                delta_calibrate_flags |= CALIBRATE_SILENT;
            }
            if (gcode->has_letter('D')) {
                //disable auto-apply values
            } else {
                delta_calibrate_flags |= CALIBRATE_AUTOSET;
            }
            calibrate_zprobe_offset(gcode->stream);
        }
    } else if (gcode->has_m) {
        switch (gcode->m) {

            case 119: {
                gcode->stream->printf("ZProbe:%d ", probe_pin.get());
                gcode->add_nl = true;
                gcode->mark_as_taken();
            }
            break;


            case 666: { // M666 - set trim for each axis in mm
                float mm[3];
                trim2mm(mm);

                if (gcode->has_letter('X')) mm[0] = gcode->get_value('X');
                if (gcode->has_letter('Y')) mm[1] = gcode->get_value('Y');
                if (gcode->has_letter('Z')) mm[2] = gcode->get_value('Z');

                int dirx = (this->home_direction[0] ? 1 : -1);
                int diry = (this->home_direction[1] ? 1 : -1);
                int dirz = (this->home_direction[2] ? 1 : -1);
                trim[0] = lround(mm[0] * steps_per_mm) * dirx; // convert back to steps
                trim[1] = lround(mm[1] * steps_per_mm) * diry;
                trim[2] = lround(mm[2] * steps_per_mm) * dirz;
                gcode->mark_as_taken();
            }
            break;            
        }
    }
}

void DeltaCalibrate::trim2mm(float *mm)
{
    int dirx = (this->home_direction[0] ? 1 : -1);
    int diry = (this->home_direction[1] ? 1 : -1);
    int dirz = (this->home_direction[2] ? 1 : -1);

    mm[0] = this->trim[0] / this->steps_per_mm * dirx; // convert to mm
    mm[1] = this->trim[1] / this->steps_per_mm * diry;
    mm[2] = this->trim[2] / this->steps_per_mm * dirz;
}

void DeltaCalibrate::send_gcode(std::string g) {
    Gcode gcode(g, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode );
}
