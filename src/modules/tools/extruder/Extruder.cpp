#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
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

Extruder::Extruder(PinName stppin) : step_pin(stppin){}

void Extruder::on_module_loaded() {
    extruder_for_irq = this;

    // Settings
    this->on_config_reload(this);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);

    // Configuration
    this->acceleration_ticker.attach_us(this, &Extruder::acceleration_tick, 1000000/this->kernel->stepper->acceleration_ticks_per_second);

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

void Extruder::on_config_reload(void* argument){
    this->microseconds_per_step_pulse = this->kernel->config->value(microseconds_per_step_pulse_ckeckusm)->by_default(5)->as_number();
}

void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = block; 
    this->start_position = this->current_position;
    this->target_position = this->start_position + ( this->current_block->millimeters * 159 ); //TODO : Get from config ( extruder_steps_per_mm )
}

void Extruder::on_block_end(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = NULL; 
}       

void Extruder::acceleration_tick(){
    // If we are currently taking care of a block            
    if( this->current_block ){
        double steps_by_acceleration_tick = this->kernel->stepper->trapezoid_adjusted_rate / 60 / this->kernel->stepper->acceleration_ticks_per_second;                
        // Get position along the block at next tick
        double next_position_ratio = ( this->kernel->stepper->step_events_completed + steps_by_acceleration_tick ) / this->kernel->stepper->current_block->steps_event_count;
        // Get wanted next position
        double next_absolute_position = ( this->target_position - this->start_position ) * next_position_ratio;
        // Get desired  speed in steps per minute to get to the next position by the next acceleration tick
        double desired_speed = ( ( next_absolute_position + this->start_position ) - this->current_position ) * double(this->kernel->stepper->acceleration_ticks_per_second) * 60L; //TODO : Replace with the actual current_position

        // Set timer
        LPC_TIM1->MR0 = ((SystemCoreClock/4)*60)/int(floor(desired_speed)); 

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
    if( this->current_position < this->target_position ){
        this->current_position++;
        this->step_pin = 1;
    }else{
        if( this->current_block == NULL ){
            //this->stepping_ticker.detach();
        }
    }
}



