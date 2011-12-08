#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Player.h"
#include "modules/robot/Block.h"
#include "modules/tools/extruder/Extruder.h"



Extruder* extruder_for_irq; // We need this global because ISRs can't be attached to an object
extern "C" void TIMER1_IRQHandler (void){
    if((LPC_TIM1->IR >> 1) & 1){
        LPC_TIM1->IR |= 1 << 1;
        extruder_for_irq->step_pin = 0;
    }
    if((LPC_TIM1->IR >> 0) & 1){
        LPC_TIM1->IR |= 1 << 0;
        extruder_for_irq->stepping_tick();
    }
}

Extruder::Extruder(PinName stppin, PinName dirpin) : step_pin(stppin), dir_pin(dirpin) {
    this->absolute_mode = true;
    this->direction     = 1;
}

void Extruder::on_module_loaded() {

    this->debug = false;

    if( this->kernel->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() == false ){ return; } 

    extruder_for_irq = this;

    // Settings
    this->on_config_reload(this);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);

    this->start_position = 0;
    this->target_position = 0;
    this->current_position = 0;
    this->current_block = NULL;

    // Start Timer1
    LPC_TIM1->MR0 = 10000; 
    LPC_TIM1->MR1 = 500;
    LPC_TIM1->MCR = 11;            // For MR0 and MR1, with no reset at MR1
    NVIC_EnableIRQ(TIMER1_IRQn);
    LPC_TIM1->TCR = 1;  
}

// Get config
void Extruder::on_config_reload(void* argument){
    this->microseconds_per_step_pulse = 5; //this->kernel->config->value(microseconds_per_step_pulse_ckeckusm)->by_default(5)->as_number();
}

// Computer extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
   
    //this->kernel->serial->printf("e: %s\r\n", gcode->command.c_str() );

    // Absolute/relative mode
    if( gcode->has_letter('M')){
        int code = gcode->get_value('M');
        if( code == 82 ){ this->absolute_mode == true; }
        if( code == 83 ){ this->absolute_mode == false; }
    } 
   
    if( gcode->has_letter('G') ){

        if( gcode->get_value('G') == 92 ){

            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E');
                this->target_position  = this->current_position;
                this->start_position   = this->current_position; 
            }            

        }else{

            // Extrusion length 
            if( gcode->has_letter('E' )){
                double extrusion_distance = gcode->get_value('E');
                //this->kernel->serial->printf("a extrusion_distance: %f, target_position: %f, new_extrusion_distance: %f, current_position: %f  \r\n", extrusion_distance, this->target_position, extrusion_distance - this->target_position, this->current_position );
                if( this->absolute_mode == true ){ 
                    extrusion_distance = extrusion_distance - this->target_position;
                }
                //this->kernel->serial->printf("b extrusion_distance: %f, target_position: %f \r\n", extrusion_distance, this->target_position );
                if( fabs(gcode->millimeters_of_travel) < 0.0001 ){
                    this->solo_mode = true;
                    this->travel_distance = extrusion_distance;
                    this->travel_ratio = 0.0;
                }else{
                    this->solo_mode = false;
                    this->travel_ratio = extrusion_distance / gcode->millimeters_of_travel; 
                    this->travel_distance = 0.0;
                } 
            // Else do not extrude
            }else{
                this->travel_ratio = 0.0;
                this->travel_distance = 0.0;
            }

        }
 
    }    
    
}



void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);

    if( fabs(this->travel_distance) > 0.001 ){
        block->take(); // In solo mode we take the block so we can move even if the stepper has nothing to do
        this->current_block = block; 
        this->start_position = this->target_position;
        this->target_position = this->start_position + this->travel_distance ;
        //this->kernel->serial->printf("bb: target_position: %f, travel_distance:%f \r\n", this->target_position, this->travel_distance );
        this->on_speed_change(this);
    }   
    if( fabs(this->travel_ratio) > 0.001 ){
        // In non-solo mode, we just follow the stepper module
        this->current_block = block; 
        this->start_position = this->target_position;
        this->target_position =  this->start_position + ( this->current_block->millimeters * this->travel_ratio );
        this->on_speed_change(this);
    } 

}

void Extruder::on_block_end(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = NULL; 
}       

void Extruder::on_speed_change(void* argument){

    // Set direction
    if( this->target_position > this->current_position ){ 
        this->direction = 1;
    }else if( this->target_position < this->current_position ){
        this->direction = -1;
    }

    if( fabs(this->travel_ratio) > 0.001 ){
 
        if( this->kernel->stepper->current_block == NULL || fabs(this->kernel->stepper->trapezoid_adjusted_rate) < 0.0001 || this->kernel->stepper->current_block->nominal_rate == 0 ){
            return;
        }

        // Get a current/nominal rate ratio
        double stepper_rate_ratio = this->kernel->stepper->trapezoid_adjusted_rate / double( this->kernel->stepper->current_block->nominal_rate ) ;
        // Get a nominal duration for this block
        double nominal_duration =  this->kernel->stepper->current_block->millimeters / ( this->kernel->stepper->current_block->nominal_speed / 60 )  ;
        // Get extrusion nominal speed
        double nominal_extrusion_speed = fabs( this->target_position - this->start_position ) / nominal_duration;
        // Get adjusted speed
        double adjusted_speed = nominal_extrusion_speed * stepper_rate_ratio;

        // Set timer
        if((adjusted_speed*1400) < 1 ){ 
            return; 
        } 
        LPC_TIM1->MR0 = ((SystemCoreClock/4))/(adjusted_speed*1400); 
        if( LPC_TIM1->MR0 < 300 ){ 
            this->kernel->serial->printf("tim1mr0 %d, adjusted_speed: %f\r\n", LPC_TIM1->MR0, adjusted_speed ); 
            LPC_TIM1->MR0 = 300; 
        }

        // In case we are trying to set the timer to a limit it has already past by
        if( LPC_TIM1->TC >= LPC_TIM1->MR0 ){
            LPC_TIM1->TCR = 3; 
            LPC_TIM1->TCR = 1; 
        }
        
        // Update Timer1    
        LPC_TIM1->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse;


    }

    if( fabs(this->travel_distance) > 0.001 ){

        // Set timer
        LPC_TIM1->MR0 = ((SystemCoreClock/4))/int(floor(1.6*1400)); 

        // In case we are trying to set the timer to a limit it has already past by
        if( LPC_TIM1->TC >= LPC_TIM1->MR0 ){
            LPC_TIM1->TCR = 3; 
            LPC_TIM1->TCR = 1; 
        }

        // Update Timer1    
        LPC_TIM1->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse;

    }

}


inline void Extruder::stepping_tick(){

    //if( fabs( this->current_position - this->target_position ) >= 0.001 ){
    if( ( this->current_position < this->target_position && this->direction == 1 ) || ( this->current_position > this->target_position && this->direction == -1 ) ){    
        this->current_position += (double(double(1)/double(1400)))*double(this->direction);
        this->dir_pin = ((this->direction > 0) ? 0 : 1);
        this->step_pin = 1;
    }else{
        // Move finished
        if( fabs(this->travel_distance) > 0.001 && this->current_block != NULL ){
            this->current_block->release();        
        } 
    }
}



