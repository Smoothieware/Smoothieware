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
   
    //this->kernel->serial->printf("gcode_exec: %s \r\n", gcode->command.c_str() );

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
                if( this->absolute_mode == true ){ 
                    extrusion_distance = extrusion_distance - this->target_position;
                }
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

    //this->kernel->serial->printf("block_begin ");
    //block->debug(this->kernel);

    if( fabs(this->travel_distance) > 0.001 ){
        block->take(); // In solo mode we take the block so we can move even if the stepper has nothing to do
        this->current_block = block; 
        //this->kernel->serial->printf("before: start:%f target:%f current:%f\r\n", this->start_position, this->target_position, this->current_position);
        this->start_position = this->target_position;
        this->target_position = this->start_position + this->travel_distance ;
        //this->kernel->serial->printf("after: start:%f target:%f current:%f\r\n", this->start_position, this->target_position, this->current_position);
        this->on_speed_change(this);
    }   
    if( fabs(this->travel_ratio) > 0.001 ){
        // In non-solo mode, we just follow the stepper module
        this->current_block = block; 
        //this->kernel->serial->printf("before: start:%f target:%f current:%f\r\n", this->start_position, this->target_position, this->current_position);
        //this->kernel->serial->printf("a:direction: %d start:%f current: %f target: %f add:%f close:%f \r\n", this->direction, this->start_position, this->current_position, this->target_position,  (double(double(1)/double(1001)))*double(this->direction),  fabs( this->current_position - this->target_position )  );
        this->start_position = this->target_position;
        this->target_position =  this->start_position + ( this->current_block->millimeters * this->travel_ratio );
        this->on_speed_change(this);
        //this->kernel->serial->printf("d:direction: %d start:%f current: %f target: %f add:%f close:%f \r\n", this->direction, this->start_position, this->current_position, this->target_position,  (double(double(1)/double(1001)))*double(this->direction),  fabs( this->current_position - this->target_position )  );
        //this->kernel->serial->printf("after: start:%f target:%f current:%f\r\n", this->start_position, this->target_position, this->current_position);
    } 

}

void Extruder::on_block_end(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = NULL; 
}       

void Extruder::on_speed_change(void* argument){


    if( this->debug ){ this->kernel->serial->printf("back in event \r\n");wait(0.1);  }

    // Set direction
    if( this->target_position > this->current_position ){ 
        this->direction = 1;
    }else if( this->target_position < this->current_position ){
        this->direction = -1;
    }

    if( fabs(this->travel_ratio) > 0.001 ){

        if( this->debug ){ 
            this->kernel->serial->printf("back in if \r\n");
            this->kernel->stepper->current_block->debug(this->kernel); 
            wait(0.1);  
        }

        // Do not change rate if stepper rate is null
        if( this->kernel->stepper->current_block == NULL || fabs(this->kernel->stepper->trapezoid_adjusted_rate) < 0.0001 || this->kernel->stepper->current_block->nominal_rate == 0 ){
                return;
            }


        if( this->debug ){ this->kernel->serial->printf("back in if2 \r\n");wait(0.1);  }

        // Get a current/nominal rate ratio
        if( fabs(this->kernel->stepper->current_block->nominal_rate) < 0.01 ){ 
            wait(1);  
        }


        if( this->debug ){ this->kernel->serial->printf("back in if3 \r\n");wait(0.1);  }

        double stepper_rate_ratio = this->kernel->stepper->trapezoid_adjusted_rate / double( this->kernel->stepper->current_block->nominal_rate ) ;
        // Get a nominal duration for this block

        
        if( this->debug ){ this->kernel->serial->printf("back in if4 \r\n");wait(0.1);  }
        
        double nominal_duration =  this->kernel->stepper->current_block->millimeters / ( this->kernel->stepper->current_block->nominal_speed / 60 )  ;
       
        bool testo = false; 
        if( this->debug ){ 
            this->kernel->serial->printf("back in if5 \r\n");wait(0.1); 
            this->debug_count--;
            if( this->debug_count == 0 ){ 
                this->debug = false; 
            } 
            testo = true;  
        }
        if( testo ){ this->kernel->serial->printf("back in if7 \r\n");wait(0.1);  }
        
        // Get extrusion nominal speed
        if( fabs(nominal_duration) < 0.001 || this->debug ){ 
            this->kernel->serial->printf("start debugging, nominal_duration: %f \r\n", double(nominal_duration));
            this->kernel->stepper->current_block->debug(this->kernel); 
            this->debug = true;
            this->debug_count = 5; 
            wait(0.1);
        }

        if( testo ){ this->kernel->serial->printf("back in if8 \r\n");wait(0.1);  }
        double nominal_extrusion_speed = fabs( this->target_position - this->start_position ) / nominal_duration;
        
        
        if( testo ){ this->kernel->serial->printf("back in if9 \r\n");wait(0.1);  }
        if( this->debug ){ this->kernel->serial->printf("nom_extr_speed: %f \r\n", double(nominal_extrusion_speed));wait(0.1);  }
        // Get adjusted speed
        double adjusted_speed = nominal_extrusion_speed * stepper_rate_ratio;


        if( testo ){ this->kernel->serial->printf("back in if10 \r\n");wait(0.1);  }

        if( this->debug ){ this->kernel->serial->printf("adj_speed a: %f \r\n", double(adjusted_speed));wait(0.1);  }
        //this->kernel->serial->printf("e:direction: %d start:%f current: %f target: %f add:%f close:%f \r\n", this->direction, this->start_position, this->current_position, this->target_position,  (double(double(1)/double(1001)))*double(this->direction),  fabs( this->current_position - this->target_position )  );
        //this->kernel->serial->printf("speed change rate:%f/%u=%f dur:%f=%f/%f nom_extr_spd:%f/%f=%f adj_speed:%f \r\n", this->kernel->stepper->trapezoid_adjusted_rate, this->kernel->stepper->current_block->nominal_rate, stepper_rate_ratio, nominal_duration, this->kernel->stepper->current_block->millimeters, this->kernel->stepper->current_block->nominal_speed / 60, ( this->target_position - this->start_position ), nominal_duration, nominal_extrusion_speed, adjusted_speed  ); 


        if( testo ){ this->kernel->serial->printf("back in if11 \r\n");wait(0.1);  }
        if( this->debug ){ this->kernel->serial->printf("adj_speed b: %f \r\n", double(adjusted_speed));wait(0.1);  }
        // Set timer
        LPC_TIM1->MR0 = ((SystemCoreClock/4))/int(floor(adjusted_speed*1000)); 

        if( testo ){ this->kernel->serial->printf("back in if12 \r\n");wait(0.1);  }
        if( this->debug ){ this->kernel->serial->printf("adj_speed c: %f \r\n", double(adjusted_speed));wait(0.1);  }
        // In case we are trying to set the timer to a limit it has already past by
        if( LPC_TIM1->TC >= LPC_TIM1->MR0 ){
            LPC_TIM1->TCR = 3; 
            LPC_TIM1->TCR = 1; 
        }

        // Update Timer1    
        LPC_TIM1->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse;

        if( testo ){ this->kernel->serial->printf("back in if13 \r\n");wait(0.1);  }
        if( this->debug ){ this->kernel->serial->printf("adj_speed d: %f \r\n", double(adjusted_speed));wait(1);  }
    }

    if( fabs(this->travel_distance) > 0.001 ){

        // Set timer
        LPC_TIM1->MR0 = ((SystemCoreClock/4))/int(floor(1000)); 

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
    if( fabs( this->current_position - this->target_position ) >= 0.001 ){
        //this->kernel->serial->printf("b:direction: %d start:%f current: %f target: %f add:%f close:%f \r\n", this->direction, this->start_position, this->current_position, this->target_position,  (double(double(1)/double(1001)))*double(this->direction),  fabs( this->current_position - this->target_position )  );
        this->current_position += (double(double(1)/double(1000)))*double(this->direction);
        //this->kernel->serial->printf("c:direction: %d start:%f current: %f target: %f add:%f close:%f \r\n", this->direction, this->start_position, this->current_position, this->target_position,  (double(double(1)/double(1001)))*double(this->direction),  fabs( this->current_position - this->target_position )  );
        this->dir_pin = ((this->direction > 0) ? 1 : 0);
        this->step_pin = 1;
    }else{
        // Move finished
        if( fabs(this->travel_distance) > 0.001 && this->current_block != NULL ){
            this->current_block->release();        
        } 
    }
}



