#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Stepper.h"
#include "proton.h"
#include "libs/nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "Block.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "libs/Pin.h"
#include "Gcode.h"

#define proton_module_enable_checksum        CHECKSUM("proton_module_enable")

Proton::Proton () {

}
Proton::~Proton() {

}

void Proton::on_module_loaded() {
    if( !THEKERNEL->config->value( proton_module_enable_checksum )->by_default(false)->as_bool() ){
        // as not needed free up resource
        delete this;
        return;
    }
	this->register_for_event(ON_GCODE_EXECUTE);
	this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
}

// 
void  Proton::on_block_end(void* argument){
   
}

// Set laser power at the beginning of a block
void Proton::on_block_begin(void* argument){
    
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Proton::on_pause(void* argument){
  
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Proton::on_play(void* argument){
 
}

// Turn laser on/off depending on received GCodes
void Proton::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_g){
        int code = gcode->g;
        if( code == 666 ){                    // G0
            THEKERNEL->streams->printf("Proton GCODE executed\n");
        }
    }
    if ( gcode->has_letter('S' )){
       
    }

}
void Proton::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_g){
        int code = gcode->g;
        if( code == 666 ){                    // G0
            THEKERNEL->streams->printf("Proton GCODE received\n");
        }
    }
    if ( gcode->has_letter('S' )){
       
    }
}

void Proton::on_speed_change(void* argument) {
	
}
